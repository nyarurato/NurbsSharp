using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.IO
{
    /// <summary>
    /// (en)Exporter for OBJ file format
    /// (ja)OBJファイルのエクスポーター
    /// </summary>
    public class OBJExporter
    {
        /// <summary>
        /// (en)Export mesh to OBJ format
        /// (ja)メッシュをOBJ形式でエクスポートする
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="stream"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public async static Task<bool> ExportAsync(Mesh mesh,Stream stream)
        {
            if (mesh == null)
                throw new ArgumentNullException(nameof(mesh));
            if(stream == null)
                throw new ArgumentNullException(nameof(stream));

            using (var writer = new StreamWriter(stream, Encoding.UTF8, 1024,true))
            {
                // Export vertices
                foreach (var vertex in mesh.Vertices)
                {
                    await writer.WriteLineAsync($"v {vertex.X} {vertex.Y} {vertex.Z}");
                }
                // Export faces
                for (int i = 0; i < mesh.Indexes.Length; i += 3)
                {
                    // OBJ format uses 1-based indexing
                    int v1 = mesh.Indexes[i] + 1;
                    int v2 = mesh.Indexes[i + 1] + 1;
                    int v3 = mesh.Indexes[i + 2] + 1;
                    await writer.WriteLineAsync($"f {v1} {v2} {v3}");
                }
            }
            
            return true;
        }

        /// <summary>
        /// Experimental
        /// (en)Export NURBS surface to OBJ format 
        /// (ja)NURBSサーフェスをOBJ形式でエクスポートする
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="stream"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public async static Task<bool> ExportAsync(NurbsSurface surface,Stream stream)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));
            if (stream == null)
                throw new ArgumentNullException(nameof(stream));

            using(var writer = new StreamWriter(stream, Encoding.UTF8, 1024, true))
            {
                writer.WriteLine("# Exported NURBS Surface - OBJ format does not natively support NURBS");
                int nU = surface.ControlPoints.Length;
                int nV = surface.ControlPoints[0].Length;
                var knotsU = surface.KnotVectorU.Knots;
                var knotsV = surface.KnotVectorV.Knots;
                // Export control points as vertices
                for (int i = 0; i < nU; i++)
                {
                    for (int j = 0; j < nV; j++)
                    {
                        var cp = surface.ControlPoints[i][j];                      
                        await writer.WriteLineAsync($"v {cp.Position.X} {cp.Position.Y} {cp.Position.Z} {cp.Weight}");
                    }
                }
                await writer.WriteLineAsync("cstype rat bspline");
                await writer.WriteLineAsync($"deg {surface.DegreeU} {surface.DegreeV}");
                await writer.WriteLineAsync($"surf {knotsU.First()} {knotsU.Last()} {knotsV.First()} {knotsV.Last()}");
                string knotsUString = string.Join(" ", knotsU.Select(k => k.ToString("G6")));
                await writer.WriteLineAsync($"param u {knotsUString}");
                string knotsVString = string.Join(" ", knotsV.Select(k => k.ToString("G6")));
                await writer.WriteLineAsync($"param v {knotsVString}");
                await writer.WriteLineAsync("end");
            }

            return true;
        }
    }
}
