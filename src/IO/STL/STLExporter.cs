using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System.IO;

namespace NurbsSharp.IO
{
    /// <summary>
    /// (en)Exporter for STL file format
    /// (ja)STLファイルのエクスポーター
    /// </summary>
    public class STLExporter
    {
        /// <summary>
        /// (en)Export mesh to STL format
        /// (ja)メッシュをSTL形式でエクスポートする
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="stream"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public async static Task<bool> ExportAsync(Mesh mesh, Stream stream)
        {
            Guard.ThrowIfNull(mesh, nameof(mesh));
            Guard.ThrowIfNull(stream, nameof(stream));

            using var writer = new StreamWriter(stream, Encoding.UTF8, 1024, true);
            await writer.WriteLineAsync("solid nurbs_mesh");
            // Export faces
            for (int i = 0; i < mesh.Indexes.Length; i += 3)
            {
                var v1 = mesh.Vertices[mesh.Indexes[i]];
                var v2 = mesh.Vertices[mesh.Indexes[i + 1]];
                var v3 = mesh.Vertices[mesh.Indexes[i + 2]];
                // Compute normal
                var normal = Vector3Double.Cross(v2 - v1, v3 - v1);
                if(normal.magnitude > 0)
                    normal = normal.normalized;
                else
                    normal = new Vector3Double(0, 0, 0);
                await writer.WriteLineAsync($"facet normal {normal.X} {normal.Y} {normal.Z}");
                await writer.WriteLineAsync("  outer loop");
                await writer.WriteLineAsync($"    vertex {v1.X} {v1.Y} {v1.Z}");
                await writer.WriteLineAsync($"    vertex {v2.X} {v2.Y} {v2.Z}");
                await writer.WriteLineAsync($"    vertex {v3.X} {v3.Y} {v3.Z}");
                await writer.WriteLineAsync("  endloop");
                await writer.WriteLineAsync("endfacet");
            }
            await writer.WriteLineAsync("endsolid nurbs_mesh");
            return true;
        }
    }
}
