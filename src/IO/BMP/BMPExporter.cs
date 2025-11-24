using System;
using System.IO;
using System.Numerics;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;

namespace NurbsSharp.IO.BMP
{
    /// <summary>
    /// (en) Exporter for BMP wireframe images
    /// (ja) BMPワイヤーフレーム画像のエクスポーター
    /// </summary>
    public class BMPExporter
    {
        /// <summary>
        /// (en) Export NURBS surface as wireframe BMP image (async)
        /// (ja) NURBSサーフェスをワイヤーフレームBMP画像として出力（非同期）
        /// </summary>
        /// <param name="surface">NURBS surface to export</param>
        /// <param name="filePath">Output file path</param>
        /// <param name="width">Image width in pixels</param>
        /// <param name="height">Image height in pixels</param>
        /// <param name="numPointsU">Number of points in U direction</param>
        /// <param name="numPointsV">Number of points in V direction</param>
        /// <param name="cameraPosition">Camera position (looks at origin), default is top-down view (0,0,10)</param>
        public static async Task ExportWireframeAsync(
            NurbsSurface surface,
            string filePath,
            int width = 800,
            int height = 800,
            int numPointsU = 20,
            int numPointsV = 20,
            Vector3Double? cameraPosition = null)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            Guard.ThrowIfNull(filePath, nameof(filePath));

            // Default camera position: top-down view
            var camPos = cameraPosition ?? new Vector3Double(0, 0, 10);

            // Tessellate surface to get mesh
            var mesh = SurfaceTessellator.Tessellate(surface, numPointsU, numPointsV);

            // Calculate 3D bounding box
            double minX = double.MaxValue, maxX = double.MinValue;
            double minY = double.MaxValue, maxY = double.MinValue;
            double minZ = double.MaxValue, maxZ = double.MinValue;

            foreach (var vertex in mesh.Vertices)
            {
                if (vertex.X < minX) minX = vertex.X;
                if (vertex.X > maxX) maxX = vertex.X;
                if (vertex.Y < minY) minY = vertex.Y;
                if (vertex.Y > maxY) maxY = vertex.Y;
                if (vertex.Z < minZ) minZ = vertex.Z;
                if (vertex.Z > maxZ) maxZ = vertex.Z;
            }

            // Add 5% padding
            double rangeX = maxX - minX;
            double rangeY = maxY - minY;
            double rangeZ = maxZ - minZ;
            double padding = 0.05;
            minX -= rangeX * padding;
            maxX += rangeX * padding;
            minY -= rangeY * padding;
            maxY += rangeY * padding;
            minZ -= rangeZ * padding;
            maxZ += rangeZ * padding;

            // Create view matrix and projection matrix
            var viewMatrix = CreateViewMatrix(camPos);
            var projectionMatrix = CreateOrthographicMatrix(
                minX, maxX,
                minY, maxY,
                0.1, 1000.0  // near, far
            );
            var viewProjectionMatrix = viewMatrix * projectionMatrix;

            // Create bitmap data (24-bit RGB, white background)
            byte[] bitmapData = new byte[width * height * 3];
            for (int i = 0; i < bitmapData.Length; i++)
            {
                bitmapData[i] = 255; // White
            }

            // Helper function to draw a line
            void DrawLine(int x0, int y0, int x1, int y1)
            {
                // Simple line drawing using Bresenham's algorithm
                int dx = Math.Abs(x1 - x0);
                int dy = Math.Abs(y1 - y0);
                int sx = x0 < x1 ? 1 : -1;
                int sy = y0 < y1 ? 1 : -1;
                int err = dx - dy;

                while (true)
                {
                    // Set pixel to black (BMP is bottom-up, so flip Y)
                    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height)
                    {
                        int flippedY = height - 1 - y0;
                        int index = (flippedY * width + x0) * 3;
                        bitmapData[index] = 0;     // B
                        bitmapData[index + 1] = 0; // G
                        bitmapData[index + 2] = 0; // R
                    }

                    if (x0 == x1 && y0 == y1) break;

                    int e2 = 2 * err;
                    if (e2 > -dy)
                    {
                        err -= dy;
                        x0 += sx;
                    }
                    if (e2 < dx)
                    {
                        err += dx;
                        y0 += sy;
                    }
                }
            }

            // Draw wireframe grid lines
            // Draw U-direction lines
            for (int i = 0; i < numPointsU; i++)
            {
                for (int j = 0; j < numPointsV - 1; j++)
                {
                    int idx0 = i * numPointsV + j;
                    int idx1 = i * numPointsV + (j + 1);

                    var (x0, y0) = Transform3DTo2D(mesh.Vertices[idx0], viewProjectionMatrix, width, height);
                    var (x1, y1) = Transform3DTo2D(mesh.Vertices[idx1], viewProjectionMatrix, width, height);
                    DrawLine(x0, y0, x1, y1);
                }
            }

            // Draw V-direction lines
            for (int j = 0; j < numPointsV; j++)
            {
                for (int i = 0; i < numPointsU - 1; i++)
                {
                    int idx0 = i * numPointsV + j;
                    int idx1 = (i + 1) * numPointsV + j;

                    var (x0, y0) = Transform3DTo2D(mesh.Vertices[idx0], viewProjectionMatrix, width, height);
                    var (x1, y1) = Transform3DTo2D(mesh.Vertices[idx1], viewProjectionMatrix, width, height);
                    DrawLine(x0, y0, x1, y1);
                }
            }

            // Write BMP file
            await WriteBMPAsync(filePath, bitmapData, width, height);
        }

        /// <summary>
        /// (en) Export NURBS curve as wireframe BMP image (async)
        /// (ja) NURBS曲線をワイヤーフレームBMP画像として出力（非同期）
        /// </summary>
        /// <param name="curve">NURBS curve to export</param>
        /// <param name="filePath">Output file path</param>
        /// <param name="width">Image width in pixels</param>
        /// <param name="height">Image height in pixels</param>
        /// <param name="numSegments">Number of segments to tessellate</param>
        /// <param name="cameraPosition">Camera position (looks at origin), default is top-down view (0,0,10)</param>
        public static async Task ExportWireframeAsync(
            NurbsCurve curve,
            string filePath,
            int width = 800,
            int height = 800,
            int numSegments = 50,
            Vector3Double? cameraPosition = null)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            Guard.ThrowIfNull(filePath, nameof(filePath));

            // Default camera position: top-down view
            var camPos = cameraPosition ?? new Vector3Double(0, 0, 10);

            // Tessellate curve to get points
            var points = CurveTessellator.Tessellate(curve, numSegments);

            // Calculate 3D bounding box
            double minX = double.MaxValue, maxX = double.MinValue;
            double minY = double.MaxValue, maxY = double.MinValue;
            double minZ = double.MaxValue, maxZ = double.MinValue;

            foreach (var point in points)
            {
                if (point.X < minX) minX = point.X;
                if (point.X > maxX) maxX = point.X;
                if (point.Y < minY) minY = point.Y;
                if (point.Y > maxY) maxY = point.Y;
                if (point.Z < minZ) minZ = point.Z;
                if (point.Z > maxZ) maxZ = point.Z;
            }

            // Add 5% padding
            double rangeX = maxX - minX;
            double rangeY = maxY - minY;
            double rangeZ = maxZ - minZ;
            double padding = 0.05;
            minX -= rangeX * padding;
            maxX += rangeX * padding;
            minY -= rangeY * padding;
            maxY += rangeY * padding;
            minZ -= rangeZ * padding;
            maxZ += rangeZ * padding;

            // Create view matrix and projection matrix
            var viewMatrix = CreateViewMatrix(camPos);
            var projectionMatrix = CreateOrthographicMatrix(
                minX, maxX,
                minY, maxY,
                0.1, 1000.0  // near, far
            );
            var viewProjectionMatrix = viewMatrix * projectionMatrix;

            // Create bitmap data (24-bit RGB, white background)
            byte[] bitmapData = new byte[width * height * 3];
            for (int i = 0; i < bitmapData.Length; i++)
            {
                bitmapData[i] = 255; // White
            }

            // Helper function to draw a line
            void DrawLine(int x0, int y0, int x1, int y1)
            {
                // Simple line drawing using Bresenham's algorithm
                int dx = Math.Abs(x1 - x0);
                int dy = Math.Abs(y1 - y0);
                int sx = x0 < x1 ? 1 : -1;
                int sy = y0 < y1 ? 1 : -1;
                int err = dx - dy;

                while (true)
                {
                    // Set pixel to black (BMP is bottom-up, so flip Y)
                    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height)
                    {
                        int flippedY = height - 1 - y0;
                        int index = (flippedY * width + x0) * 3;
                        bitmapData[index] = 0;     // B
                        bitmapData[index + 1] = 0; // G
                        bitmapData[index + 2] = 0; // R
                    }

                    if (x0 == x1 && y0 == y1) break;

                    int e2 = 2 * err;
                    if (e2 > -dy)
                    {
                        err -= dy;
                        x0 += sx;
                    }
                    if (e2 < dx)
                    {
                        err += dx;
                        y0 += sy;
                    }
                }
            }

            // Draw curve as connected line segments
            for (int i = 0; i < points.Count - 1; i++)
            {
                var (x0, y0) = Transform3DTo2D(points[i], viewProjectionMatrix, width, height);
                var (x1, y1) = Transform3DTo2D(points[i + 1], viewProjectionMatrix, width, height);
                DrawLine(x0, y0, x1, y1);
            }

            // Write BMP file
            await WriteBMPAsync(filePath, bitmapData, width, height);
        }

        /// <summary>
        /// (en) Create view matrix from camera position looking at origin
        /// (ja) カメラ位置から原点を見るビュー行列を作成
        /// </summary>
        private static Matrix4x4 CreateViewMatrix(Vector3Double position)
        {
            var pos = position.ToVector3();
            var target = Vector3.Zero;  // Always look at origin
            var up = new Vector3(0, 0, 1);  // Z-axis up
            return Matrix4x4.CreateLookAt(pos, target, up);
        }

        /// <summary>
        /// (en) Create orthographic projection matrix
        /// (ja) 正投影行列を作成
        /// </summary>
        private static Matrix4x4 CreateOrthographicMatrix(
            double left, double right,
            double bottom, double top,
            double near, double far)
        {
            return Matrix4x4.CreateOrthographic(
                (float)(right - left),
                (float)(top - bottom),
                (float)near,
                (float)far
            );
        }

        /// <summary>
        /// (en) Transform 3D point to 2D screen coordinates
        /// (ja) 3D点を2Dスクリーン座標に変換
        /// </summary>
        private static (int x, int y) Transform3DTo2D(
            Vector3Double point,
            Matrix4x4 viewProjectionMatrix,
            int width,
            int height)
        {
            var vec3 = point.ToVector3();
            var transformed = Vector3.Transform(vec3, viewProjectionMatrix);

            // NDC (-1 to 1) to screen coordinates (0 to width/height)
            int x = (int)((transformed.X + 1.0f) * 0.5f * width);
            int y = (int)((1.0f - transformed.Y) * 0.5f * height);

            return (x, y);
        }

        /// <summary>
        /// (en) Write BMP file with given bitmap data (async)
        /// (ja) ビットマップデータをBMPファイルに書き込む（非同期）
        /// </summary>
        /// <param name="filePath">Output file path</param>
        /// <param name="bitmapData">24-bit RGB bitmap data</param>
        /// <param name="width">Image width</param>
        /// <param name="height">Image height</param>
        private static async Task WriteBMPAsync(string filePath, byte[] bitmapData, int width, int height)
        {
            // BMP format: https://en.wikipedia.org/wiki/BMP_file_format
            // Row size must be a multiple of 4 bytes
            int rowSize = ((width * 3 + 3) / 4) * 4;
            int pixelDataSize = rowSize * height;

            using var fs = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.None, 4096, useAsync: true);
            // BMP Header (14 bytes)
            await fs.WriteAsync(("BM"u8.ToArray()).AsMemory(0, 2));
            await WriteInt32Async(fs, 54 + pixelDataSize); // File size
            await WriteInt16Async(fs, 0); // Reserved
            await WriteInt16Async(fs, 0); // Reserved
            await WriteInt32Async(fs, 54); // Pixel data offset

            // DIB Header (BITMAPINFOHEADER, 40 bytes)
            await WriteInt32Async(fs, 40); // Header size
            await WriteInt32Async(fs, width);
            await WriteInt32Async(fs, height);
            await WriteInt16Async(fs, 1); // Color planes
            await WriteInt16Async(fs, 24); // Bits per pixel
            await WriteInt32Async(fs, 0); // Compression (BI_RGB)
            await WriteInt32Async(fs, pixelDataSize); // Image size
            await WriteInt32Async(fs, 2835); // X pixels per meter (72 DPI)
            await WriteInt32Async(fs, 2835); // Y pixels per meter (72 DPI)
            await WriteInt32Async(fs, 0); // Colors in palette
            await WriteInt32Async(fs, 0); // Important colors

            // Pixel data (bottom-up, each row padded to multiple of 4 bytes)
            byte[] paddingBytes = new byte[3]; // Max padding is 3 bytes
            for (int y = 0; y < height; y++)
            {
                int rowStart = y * width * 3;
                int rowLength = width * 3;
                await fs.WriteAsync(bitmapData.AsMemory(rowStart, rowLength));

                // Write padding bytes
                int padding = rowSize - width * 3;
                if (padding > 0)
                {
                    await fs.WriteAsync(paddingBytes.AsMemory(0, padding));
                }
            }
        }

        /// <summary>
        /// Helper method to write int32 to stream asynchronously
        /// </summary>
        private static async Task WriteInt32Async(Stream stream, int value)
        {
            byte[] bytes = BitConverter.GetBytes(value);
            await stream.WriteAsync(bytes.AsMemory(0, 4));
        }

        /// <summary>
        /// Helper method to write int16 to stream asynchronously
        /// </summary>
        private static async Task WriteInt16Async(Stream stream, short value)
        {
            byte[] bytes = BitConverter.GetBytes(value);
            await stream.WriteAsync(bytes.AsMemory(0, 2));
        }
    }
}
