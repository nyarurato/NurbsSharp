using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;

namespace NurbsSharp.Geometry
{
    /// <summary>
    /// (en)Triangle Mesh Data Structure
    /// (ja)三角形メッシュのデータ構造
    /// </summary>
    public class Mesh : IGeometry
    {
        public Vector3Double[] Vertices { get; set; }
        public int[] Indexes { get; set; }
        public Mesh()
        {
            Vertices = Array.Empty<Vector3Double>();
            Indexes = Array.Empty<int>();
        }

        public Mesh(Vector3Double[] vertices, int[] indexes)
        {
            Vertices = vertices ?? throw new ArgumentNullException(nameof(vertices));
            Indexes = indexes ?? throw new ArgumentNullException(nameof(indexes));
            Validate();
        }

        bool Validate()
        {
            if (Indexes.Length % 3 != 0)
                throw new Exception("Invalid mesh: Indexes length is not multiple of 3.");
            foreach (var index in Indexes)
            {
                if (index < 0 || index >= Vertices.Length)
                    return false;
            }
            return true;
        }
    }
}
