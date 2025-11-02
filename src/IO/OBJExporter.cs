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
    public class OBJExporter
    {
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
    }
}
