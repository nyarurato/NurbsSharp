using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Provides ray-mesh intersection tests using Möller-Trumbore algorithm
    /// (ja) Möller-Trumboreアルゴリズムを使用したレイ-メッシュ交差判定を提供
    /// </summary>
    public static class RayMeshIntersector
    {
        /// <summary>
        /// (en) Epsilon for intersection tests
        /// (ja) 交差判定の許容誤差
        /// </summary>
        private const double Epsilon = 1e-10;

        /// <summary>
        /// (en) Test if a ray intersects a triangle using Möller-Trumbore algorithm
        /// (ja) Möller-Trumboreアルゴリズムを使用してレイが三角形と交差するか判定
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="v0">First vertex of the triangle</param>
        /// <param name="v1">Second vertex of the triangle</param>
        /// <param name="v2">Third vertex of the triangle</param>
        /// <param name="intersection">Intersection result (if intersects)</param>
        /// <returns>True if the ray intersects the triangle</returns>
        /// <remarks>
        /// (en) This implementation uses the Möller-Trumbore ray-triangle intersection algorithm. Time complexity: O(1).
        ///      The algorithm computes barycentric coordinates directly.
        /// (ja) この実装はMöller-Trumboreレイ-三角形交差アルゴリズムを使用します。時間計算量: O(1)。
        ///      重心座標を直接計算します。
        /// </remarks>
        public static bool IntersectsTriangle(Ray ray, Vector3Double v0, Vector3Double v1, Vector3Double v2, out RayMeshIntersection intersection)
        {
            //https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
            intersection = default;

            // Compute edge vectors
            Vector3Double edge1 = v1 - v0;
            Vector3Double edge2 = v2 - v0;

            // Begin calculating determinant - also used to calculate U parameter
            Vector3Double h = Vector3Double.Cross(ray.Direction, edge2);
            double a = Vector3Double.Dot(edge1, h);

            // If determinant is near zero, ray lies in plane of triangle
            if (Math.Abs(a) < Epsilon)
                return false;

            double f = 1.0 / a;
            Vector3Double s = ray.Origin - v0;
            double u = f * Vector3Double.Dot(s, h);

            // Check if intersection is outside triangle (u parameter)
            if (u < 0.0 || u > 1.0)
                return false;

            Vector3Double q = Vector3Double.Cross(s, edge1);
            double v = f * Vector3Double.Dot(ray.Direction, q);

            // Check if intersection is outside triangle (v parameter)
            if (v < 0.0 || u + v > 1.0)
                return false;

            // Compute t to find where the intersection point is on the line
            double t = f * Vector3Double.Dot(edge2, q);

            // Ray intersection (t > epsilon to avoid self-intersection)
            if (t > Epsilon)
            {
                Vector3Double point = ray.PointAt(t);
                intersection = new RayMeshIntersection(t, -1, u, v, point);
                return true;
            }

            // Line intersection but not ray intersection
            return false;
        }

        /// <summary>
        /// (en) Find the FIRST intersection between a ray and a mesh using BVH acceleration
        /// (ja) BVH加速を使用してレイとメッシュの最初の交点を見つける
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="mesh">Mesh to test against</param>
        /// <param name="intersection">First intersection result (if any)</param>
        /// <returns>True if the ray intersects the mesh</returns>
        public static bool Intersects(Ray ray, Mesh mesh, out RayMeshIntersection intersection)
        {
            intersection = default;

            // Early out: empty mesh
            if (mesh.Vertices.Length == 0 || mesh.Indexes.Length == 0)
                return false;

            // Early out: check bounding box first
            if (!RayBoxIntersector.Intersects(ray, mesh.BoundingBox))
                return false;

            // Build BVH
            BVHNode bvh = BVHBuilder.Build(mesh);

            // Traverse BVH
            double closestT = double.PositiveInfinity;
            RayMeshIntersection closestIntersection = default;
            bool found = IntersectBVH(ray, bvh, mesh, ref closestT, ref closestIntersection);

            if (found)
            {
                intersection = closestIntersection;
                return true;
            }

            return false;
        }

        /// <summary>
        /// (en) Traverse BVH tree to find closest ray-mesh intersection
        /// (ja) BVHツリーをトラバースして最近接のレイ-メッシュ交差を見つける
        /// </summary>
        private static bool IntersectBVH(Ray ray, BVHNode node, Mesh mesh, ref double closestT, ref RayMeshIntersection closestIntersection)
        {
            // Check if ray intersects node's bounding box
            if (!RayBoxIntersector.Intersects(ray, node.Bounds))
                return false;

            if (node.IsLeaf)
            {
                // Test all triangles in this leaf
                bool foundInLeaf = false;
                if (node.TriangleIndices != null)
                {
                    foreach (int triangleIndex in node.TriangleIndices)
                    {
                        int idx0 = mesh.Indexes[triangleIndex * 3];
                        int idx1 = mesh.Indexes[triangleIndex * 3 + 1];
                        int idx2 = mesh.Indexes[triangleIndex * 3 + 2];

                        Vector3Double v0 = mesh.Vertices[idx0];
                        Vector3Double v1 = mesh.Vertices[idx1];
                        Vector3Double v2 = mesh.Vertices[idx2];

                        if (IntersectsTriangle(ray, v0, v1, v2, out RayMeshIntersection hit))
                        {
                            if (hit.T < closestT)
                            {
                                closestT = hit.T;
                                closestIntersection = hit;
                                closestIntersection.TriangleIndex = triangleIndex;
                                foundInLeaf = true;
                            }
                        }
                    }
                }
                return foundInLeaf;
            }
            else
            {
                // Internal node - traverse both children
                bool foundLeft = false;
                bool foundRight = false;

                if (node.Left != null)
                    foundLeft = IntersectBVH(ray, node.Left, mesh, ref closestT, ref closestIntersection);
                
                if (node.Right != null)
                    foundRight = IntersectBVH(ray, node.Right, mesh, ref closestT, ref closestIntersection);

                return foundLeft || foundRight;
            }
        }


        /// <summary>
        /// (en) Find all intersections between a ray and a mesh using BVH acceleration
        /// (ja) BVH加速を使用してレイとメッシュの全ての交点を見つける
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="mesh">Mesh to test against</param>
        /// <returns>List of all intersections, sorted by distance from ray origin</returns>
        public static List<RayMeshIntersection> IntersectAll(Ray ray, Mesh mesh)
        {
            var intersections = new List<RayMeshIntersection>();

            // Early out: empty mesh
            if (mesh.Vertices.Length == 0 || mesh.Indexes.Length == 0)
                return intersections;

            // Early out: check bounding box first
            if (!RayBoxIntersector.Intersects(ray, mesh.BoundingBox))
                return intersections;

            // Build BVH
            BVHNode bvh = BVHBuilder.Build(mesh);

            // Traverse BVH
            IntersectBVHAll(ray, bvh, mesh, intersections);

            // Sort by distance from ray origin
            intersections.Sort((a, b) => a.T.CompareTo(b.T));

            return intersections;
        }

        /// <summary>
        /// (en) Traverse BVH tree to find all ray-mesh intersections
        /// (ja) BVHツリーをトラバースしてすべてのレイ-メッシュ交差を見つける
        /// </summary>
        private static void IntersectBVHAll(Ray ray, BVHNode node, Mesh mesh, List<RayMeshIntersection> intersections)
        {
            // Check if ray intersects node's bounding box
            if (!RayBoxIntersector.Intersects(ray, node.Bounds))
                return;

            if (node.IsLeaf)
            {
                // Test all triangles in this leaf
                if (node.TriangleIndices != null)
                {
                    foreach (int triangleIndex in node.TriangleIndices)
                    {
                        int idx0 = mesh.Indexes[triangleIndex * 3];
                        int idx1 = mesh.Indexes[triangleIndex * 3 + 1];
                        int idx2 = mesh.Indexes[triangleIndex * 3 + 2];

                        Vector3Double v0 = mesh.Vertices[idx0];
                        Vector3Double v1 = mesh.Vertices[idx1];
                        Vector3Double v2 = mesh.Vertices[idx2];

                        if (IntersectsTriangle(ray, v0, v1, v2, out RayMeshIntersection hit))
                        {
                            hit.TriangleIndex = triangleIndex;
                            intersections.Add(hit);
                        }
                    }
                }
            }
            else
            {
                // Internal node - traverse both children
                if (node.Left != null)
                    IntersectBVHAll(ray, node.Left, mesh, intersections);
                
                if (node.Right != null)
                    IntersectBVHAll(ray, node.Right, mesh, intersections);
            }
        }


        /// <summary>
        /// (en) Test if a ray intersects a mesh using BVH acceleration (boolean only, fastest)
        /// (ja) BVH加速を使用してレイがメッシュと交差するか判定（真偽値のみ、最速）
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="mesh">Mesh to test against</param>
        /// <returns>True if the ray intersects any triangle in the mesh</returns>
        public static bool IntersectsAny(Ray ray, Mesh mesh)
        {
            // Early out: empty mesh
            if (mesh.Vertices.Length == 0 || mesh.Indexes.Length == 0)
                return false;

            // Early out: check bounding box first
            if (!RayBoxIntersector.Intersects(ray, mesh.BoundingBox))
                return false;

            // Build BVH
            BVHNode bvh = BVHBuilder.Build(mesh);

            // Traverse BVH with early exit
            return IntersectBVHAny(ray, bvh, mesh);
        }

        /// <summary>
        /// (en) Traverse BVH tree to check if ray intersects any triangle (early exit)
        /// (ja) BVHツリーをトラバースしてレイが任意の三角形と交差するか判定（早期終了）
        /// </summary>
        private static bool IntersectBVHAny(Ray ray, BVHNode node, Mesh mesh)
        {
            // Check if ray intersects node's bounding box
            if (!RayBoxIntersector.Intersects(ray, node.Bounds))
                return false;

            if (node.IsLeaf)
            {
                // Test all triangles in this leaf
                if (node.TriangleIndices != null)
                {
                    foreach (int triangleIndex in node.TriangleIndices)
                    {
                        int idx0 = mesh.Indexes[triangleIndex * 3];
                        int idx1 = mesh.Indexes[triangleIndex * 3 + 1];
                        int idx2 = mesh.Indexes[triangleIndex * 3 + 2];

                        Vector3Double v0 = mesh.Vertices[idx0];
                        Vector3Double v1 = mesh.Vertices[idx1];
                        Vector3Double v2 = mesh.Vertices[idx2];

                        if (IntersectsTriangle(ray, v0, v1, v2, out _))
                            return true; // Early exit on first hit
                    }
                }
                return false;
            }
            else
            {
                // Internal node - traverse children with early exit
                if (node.Left != null && IntersectBVHAny(ray, node.Left, mesh))
                    return true;
                
                if (node.Right != null && IntersectBVHAny(ray, node.Right, mesh))
                    return true;

                return false;
            }
        }
    }

    /// <summary>
    /// (en) Represents a ray-mesh intersection result
    /// (ja) レイ-メッシュ交差結果を表す
    /// </summary>
    /// <remarks>
    /// Constructor
    /// </remarks>
    public struct RayMeshIntersection(double t, int triangleIndex, double u, double v, Vector3Double point)
    {
        /// <summary>
        /// (en) Parameter along the ray where intersection occurs
        /// (ja) 交差が発生するレイ上のパラメータ
        /// </summary>
        public double T { get; set; } = t;

        /// <summary>
        /// (en) Index of the intersected triangle (offset into Indexes array / 3)
        /// (ja) 交差した三角形のインデックス（Indexes配列のオフセット / 3）
        /// </summary>
        /// <remarks>calculating by only triangle, it will be set -1</remarks>
        public int TriangleIndex { get; set; } = triangleIndex;

        /// <summary>
        /// (en) Barycentric coordinates of the intersection point (u, v)
        /// (ja) 交点の重心座標 (u, v)
        /// </summary>
        /// <remarks>
        /// (en) The intersection point can be computed as: P = (1-u-v)*V0 + u*V1 + v*V2
        /// (ja) 交点は次のように計算できます: P = (1-u-v)*V0 + u*V1 + v*V2
        /// </remarks>
        public double U { get; set; } = u;

        /// <summary>
        /// (en) V component of barycentric coordinates
        /// (ja) 重心座標のV成分
        /// </summary>
        public double V { get; set; } = v;

        /// <summary>
        /// (en) Intersection point in world coordinates
        /// (ja) ワールド座標での交点
        /// </summary>
        public Vector3Double Point { get; set; } = point;
    }

}
