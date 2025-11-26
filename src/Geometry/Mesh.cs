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
        BoundingBox? _boundingBox;
        /// <summary>
        /// (en) Array of vertices
        /// (ja) 頂点の配列
        /// </summary>
        public Vector3Double[] Vertices { get; set; }
        /// <summary>
        /// (en) Array of triangle indexes (each consecutive 3 integers represent one triangle)
        /// (ja) 三角形のインデックスの配列（連続する3つの整数が1つの三角形を表す）
        /// </summary>
        public int[] Indexes { get; set; }

        /// <summary>
        /// Bounding box by control points
        /// </summary>
        public BoundingBox BoundingBox
        {
            get
            {
                _boundingBox ??= BoundingBox.FromPoints(Vertices);
                return _boundingBox.Value;
            }
        }

        /// <summary>
        /// Constructor
        /// </summary>
        public Mesh()
        {
            Vertices = Array.Empty<Vector3Double>();
            Indexes = Array.Empty<int>();
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="indexes"></param>
        /// <exception cref="ArgumentNullException"></exception>
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
