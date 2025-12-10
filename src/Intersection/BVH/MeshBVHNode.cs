using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Node in a Bounding Volume Hierarchy (BVH) tree for accelerating ray-mesh intersection tests
    /// (ja) レイ-メッシュ交差判定を高速化するためのBVH（境界ボリューム階層）木のノード
    /// </summary>
    /// <remarks>
    /// (en) BVH is a tree structure that spatially partitions triangles to enable fast ray intersection tests.
    ///      Each node contains a bounding box. Leaf nodes contain triangle indices, while internal nodes
    ///      have two children that partition the space.
    /// (ja) BVHは三角形を空間的に分割して高速なレイ交差判定を可能にする木構造です。
    ///      各ノードは境界ボックスを持ちます。葉ノードは三角形インデックスを含み、内部ノードは
    ///      空間を分割する2つの子ノードを持ちます。
    /// </remarks>
    public class MeshBVHNode
    {
        /// <summary>
        /// (en) Bounding box encompassing all triangles in this node's subtree
        /// (ja) このノードのサブツリー内のすべての三角形を包含する境界ボックス
        /// </summary>
        public BoundingBox Bounds { get; }

        /// <summary>
        /// (en) Left child node (null for leaf nodes)
        /// (ja) 左の子ノード（葉ノードの場合はnull）
        /// </summary>
        public MeshBVHNode? Left { get; }

        /// <summary>
        /// (en) Right child node (null for leaf nodes)
        /// (ja) 右の子ノード（葉ノードの場合はnull）
        /// </summary>
        public MeshBVHNode? Right { get; }

        /// <summary>
        /// (en) Triangle indices contained in this leaf node (null for internal nodes)
        /// (ja) この葉ノードに含まれる三角形インデックス（内部ノードの場合はnull）
        /// </summary>
        public int[]? TriangleIndices { get; }

        /// <summary>
        /// (en) Whether this node is a leaf node
        /// (ja) このノードが葉ノードかどうか
        /// </summary>
        public bool IsLeaf => TriangleIndices != null;

        /// <summary>
        /// (en) Constructor for leaf nodes
        /// (ja) 葉ノード用のコンストラクタ
        /// </summary>
        /// <param name="bounds">Bounding box for this leaf</param>
        /// <param name="triangleIndices">Triangle indices contained in this leaf</param>
        public MeshBVHNode(BoundingBox bounds, int[] triangleIndices)
        {
            Bounds = bounds;
            TriangleIndices = triangleIndices;
            Left = null;
            Right = null;
        }

        /// <summary>
        /// (en) Constructor for internal nodes
        /// (ja) 内部ノード用のコンストラクタ
        /// </summary>
        /// <param name="bounds">Bounding box encompassing both children</param>
        /// <param name="left">Left child node</param>
        /// <param name="right">Right child node</param>
        public MeshBVHNode(BoundingBox bounds, MeshBVHNode left, MeshBVHNode right)
        {
            Bounds = bounds;
            Left = left;
            Right = right;
            TriangleIndices = null;
        }
    }

    /// <summary>
    /// (en) Helper class for building BVH trees
    /// (ja) BVHツリー構築用のヘルパークラス
    /// </summary>
    public static class BVHBuilder
    {
        /// <summary>
        /// (en) Maximum number of triangles in a leaf node
        /// (ja) 葉ノード内の最大三角形数
        /// </summary>
        private const int MaxTrianglesPerLeaf = 4;

        /// <summary>
        /// (en) Information about a triangle for BVH construction
        /// (ja) BVH構築用の三角形情報
        /// </summary>
        private struct TriangleInfo
        {
            public int Index;
            public BoundingBox Bounds;
            public Vector3Double Centroid;
        }

        /// <summary>
        /// (en) Build a BVH tree from mesh triangles
        /// (ja) メッシュの三角形からBVHツリーを構築
        /// </summary>
        /// <param name="mesh">Mesh to build BVH for</param>
        /// <returns>Root node of the BVH tree</returns>
        public static MeshBVHNode Build(Mesh mesh)
        {
            if (mesh.Indexes.Length == 0)
            {
                // Empty mesh - create a single leaf node with empty bounds
                return new MeshBVHNode(mesh.BoundingBox, Array.Empty<int>());
            }

            // Build triangle information
            int triangleCount = mesh.Indexes.Length / 3;
            var triangleInfos = new TriangleInfo[triangleCount];

            for (int i = 0; i < triangleCount; i++)
            {
                int idx0 = mesh.Indexes[i * 3];
                int idx1 = mesh.Indexes[i * 3 + 1];
                int idx2 = mesh.Indexes[i * 3 + 2];

                Vector3Double v0 = mesh.Vertices[idx0];
                Vector3Double v1 = mesh.Vertices[idx1];
                Vector3Double v2 = mesh.Vertices[idx2];

                BoundingBox bounds = BoundingBox.FromPoints(new[] { v0, v1, v2 });
                Vector3Double centroid = new Vector3Double(
                    (v0.X + v1.X + v2.X) / 3.0,
                    (v0.Y + v1.Y + v2.Y) / 3.0,
                    (v0.Z + v1.Z + v2.Z) / 3.0
                );

                triangleInfos[i] = new TriangleInfo
                {
                    Index = i,
                    Bounds = bounds,
                    Centroid = centroid
                };
            }

            return BuildRecursive(triangleInfos, 0, triangleCount);
        }

        /// <summary>
        /// (en) Recursively build BVH tree using Surface Area Heuristic
        /// (ja) Surface Area Heuristicを使用してBVHツリーを再帰的に構築
        /// </summary>
        private static MeshBVHNode BuildRecursive(TriangleInfo[] triangles, int start, int count)
        {
            // Compute bounding box for this node
            BoundingBox nodeBounds = triangles[start].Bounds;
            for (int i = 1; i < count; i++)
            {
                nodeBounds = nodeBounds.Union(triangles[start + i].Bounds);
            }

            // Create leaf node if few enough triangles
            if (count <= MaxTrianglesPerLeaf)
            {
                int[] indices = new int[count];
                for (int i = 0; i < count; i++)
                {
                    indices[i] = triangles[start + i].Index;
                }
                return new MeshBVHNode(nodeBounds, indices);
            }

            // Find best split using SAH (Surface Area Heuristic)
            int bestAxis = 0;
            int bestSplitIndex = count / 2;
            double bestCost = double.PositiveInfinity;

            // Compute centroid bounds
            Vector3Double minCentroid = triangles[start].Centroid;
            Vector3Double maxCentroid = triangles[start].Centroid;
            for (int i = 1; i < count; i++)
            {
                Vector3Double c = triangles[start + i].Centroid;
                minCentroid = new Vector3Double(
                    Math.Min(minCentroid.X, c.X),
                    Math.Min(minCentroid.Y, c.Y),
                    Math.Min(minCentroid.Z, c.Z)
                );
                maxCentroid = new Vector3Double(
                    Math.Max(maxCentroid.X, c.X),
                    Math.Max(maxCentroid.Y, c.Y),
                    Math.Max(maxCentroid.Z, c.Z)
                );
            }

            // Try each axis
            for (int axis = 0; axis < 3; axis++)
            {
                // Skip degenerate axes
                double extent = axis == 0 ? (maxCentroid.X - minCentroid.X) :
                               axis == 1 ? (maxCentroid.Y - minCentroid.Y) :
                                          (maxCentroid.Z - minCentroid.Z);
                if (extent < 1e-10) continue;

                // Sort by centroid along this axis
                Array.Sort(triangles, start, count, Comparer<TriangleInfo>.Create((a, b) =>
                {
                    double aVal = axis == 0 ? a.Centroid.X : axis == 1 ? a.Centroid.Y : a.Centroid.Z;
                    double bVal = axis == 0 ? b.Centroid.X : axis == 1 ? b.Centroid.Y : b.Centroid.Z;
                    return aVal.CompareTo(bVal);
                }));

                // Try different split positions using SAH
                const int numBuckets = 12;
                for (int bucket = 1; bucket < numBuckets; bucket++)
                {
                    int splitIndex = (count * bucket) / numBuckets;
                    if (splitIndex == 0 || splitIndex == count) continue;

                    // Compute cost for this split
                    BoundingBox leftBounds = triangles[start].Bounds;
                    for (int i = 1; i < splitIndex; i++)
                    {
                        leftBounds = leftBounds.Union(triangles[start + i].Bounds);
                    }

                    BoundingBox rightBounds = triangles[start + splitIndex].Bounds;
                    for (int i = splitIndex + 1; i < count; i++)
                    {
                        rightBounds = rightBounds.Union(triangles[start + i].Bounds);
                    }

                    double leftArea = SurfaceArea(leftBounds);
                    double rightArea = SurfaceArea(rightBounds);
                    double cost = leftArea * splitIndex + rightArea * (count - splitIndex);

                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        bestAxis = axis;
                        bestSplitIndex = splitIndex;
                    }
                }
            }

            // Sort by best axis
            Array.Sort(triangles, start, count, Comparer<TriangleInfo>.Create((a, b) =>
            {
                double aVal = bestAxis == 0 ? a.Centroid.X : bestAxis == 1 ? a.Centroid.Y : a.Centroid.Z;
                double bVal = bestAxis == 0 ? b.Centroid.X : bestAxis == 1 ? b.Centroid.Y : b.Centroid.Z;
                return aVal.CompareTo(bVal);
            }));

            // Build child nodes
            MeshBVHNode left = BuildRecursive(triangles, start, bestSplitIndex);
            MeshBVHNode right = BuildRecursive(triangles, start + bestSplitIndex, count - bestSplitIndex);

            return new MeshBVHNode(nodeBounds, left, right);
        }

        /// <summary>
        /// (en) Compute surface area of a bounding box
        /// (ja) 境界ボックスの表面積を計算
        /// </summary>
        private static double SurfaceArea(BoundingBox box)
        {
            Vector3Double size = box.Max - box.Min;
            return 2.0 * (size.X * size.Y + size.Y * size.Z + size.Z * size.X);
        }
    }
}
