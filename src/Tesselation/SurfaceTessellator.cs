using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Tesselation
{
    /// <summary>
    /// (en)Tessellator for NURBS surfaces
    /// (ja)NURBSサーフェスのテッセレーター
    /// </summary>
    public class SurfaceTessellator
    {
        /// <summary>
        /// (en)Tessellate NURBS surface into triangle mesh
        /// (ja)NURBSサーフェスを三角形メッシュにする
        /// </summary>
        /// <param name="surface">NURBS surface to tessellate</param>
        /// <param name="numPointsU">Number of sample points in U direction</param>
        /// <param name="numPointsV">Number of sample points in V direction</param>
        /// <returns>Tessellated triangle mesh</returns>
        public static Mesh Tessellate(NurbsSurface surface, int numPointsU, int numPointsV)
        {
            Vector3Double[] vertices = new Vector3Double[numPointsU * numPointsV];
            int[] indexes = new int[(numPointsU - 1) * (numPointsV - 1) * 6];
            double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
            double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
            double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];
            bool useParallel = numPointsU * numPointsV >= 20000; // threshold for parallelization

            // Generate vertices (parallelized for performance)
            if (useParallel)
            {
                Parallel.For(0, numPointsU, i =>
                {
                    double u = uStart + (uEnd - uStart) * i / (numPointsU - 1);
                    for (int j = 0; j < numPointsV; j++)
                    {
                        double v = vStart + (vEnd - vStart) * j / (numPointsV - 1);
                        var point = surface.GetPos(u, v);
                        vertices[i * numPointsV + j] = point;
                    }
                });
            }
            else
            {
                for (int i = 0; i < numPointsU; i++)
                {
                    double u = uStart + (uEnd - uStart) * i / (numPointsU - 1);
                    for (int j = 0; j < numPointsV; j++)
                    {
                        double v = vStart + (vEnd - vStart) * j / (numPointsV - 1);
                        var point = surface.GetPos(u, v);
                        vertices[i * numPointsV + j] = point;
                    }
                }
            }

            int index = 0;
            for (int i = 0; i < numPointsU - 1; i++)
            {
                for (int j = 0; j < numPointsV - 1; j++)
                {
                    int v0 = i * numPointsV + j;
                    int v1 = (i + 1) * numPointsV + j;
                    int v2 = (i + 1) * numPointsV + (j + 1);
                    int v3 = i * numPointsV + (j + 1);
                    // First triangle
                    indexes[index++] = v0;
                    indexes[index++] = v1;
                    indexes[index++] = v2;
                    // Second triangle
                    indexes[index++] = v0;
                    indexes[index++] = v2;
                    indexes[index++] = v3;
                }
            }
            return new Mesh(vertices, indexes);

        }

        /// <summary>
        /// (en)Adaptive tessellation for NURBS surfaces using curvature-based subdivision.
        /// (ja)曲率ベースの細分化を使用したNURBSサーフェスの適応テッセレーション
        /// </summary>
        /// <param name="surface">NURBS surface to tessellate</param>
        /// <param name="tolerance">(en)Maximum curvature threshold (1/radius). Subdivides when max principal curvature exceeds this value; (ja)最大曲率の閾値（1/半径）。主曲率がこの値を超えると細分化</param>
        /// <param name="maxDepth">(en)Maximum recursion depth for subdivision; (ja)細分化の最大深さ</param>
        /// <returns>Adaptively tessellated triangle mesh</returns>
        public static Mesh TessellateAdaptive(NurbsSurface surface, double tolerance, int maxDepth = 8)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            Guard.ThrowIfNegativeOrZero(tolerance, nameof(tolerance));

            double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
            double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
            double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

            // Pre-allocate with estimated capacity for better performance.
            // Upper bound grows as ~4^depth; keep it conservative to avoid huge allocations.
            int estimatedVertexCount = (int)Math.Pow(4, Math.Min(maxDepth, 6));
            var vertices = new List<Vector3Double>(estimatedVertexCount);
            var indexes = new List<int>(estimatedVertexCount * 6);

            // Avoid string keys/allocations: use bitwise-equality on (u,v) doubles.
            // Subdivision uses repeated midpoints, so exact-bit keys are reliable and fast.
            var indexMap = new Dictionary<ParamKey, int>(estimatedVertexCount, ParamKeyComparer.Instance);
            var curvatureCache = new Dictionary<ParamKey, double>(estimatedVertexCount, ParamKeyComparer.Instance);

            int AddVertex(double u, double v)
            {
                var key = new ParamKey(u, v);
                if (indexMap.TryGetValue(key, out var idx))
                    return idx;
                var p = surface.GetPos(u, v);
                idx = vertices.Count;
                vertices.Add(p);
                indexMap.Add(key, idx);
                return idx;
            }

            void AddTriangleByParams((double u, double v) a, (double u, double v) b, (double u, double v) c)
            {
                var ia = AddVertex(a.u, a.v);
                var ib = AddVertex(b.u, b.v);
                var ic = AddVertex(c.u, c.v);
                indexes.Add(ia);
                indexes.Add(ib);
                indexes.Add(ic);
            }

            void Subdivide(double u0, double u1, double v0, double v1, int depth)
            {
                double um = (u0 + u1) / 2.0;
                double vm = (v0 + v1) / 2.0;

                double GetMaxCurvatureCached(double u, double v)
                {
                    var key = new ParamKey(u, v);
                    if (curvatureCache.TryGetValue(key, out var cached))
                        return cached;

                    double value;
                    try
                    {
                        var (k1, k2) = surface.GetPrincipalCurvatures(u, v);
                        value = Math.Max(Math.Abs(k1), Math.Abs(k2));
                    }
                    catch
                    {
                        // Fallback: use Gaussian curvature (product of principal curvatures)
                        try
                        {
                            var (mean, gaussian) = surface.GetMeanAndGaussianCurvatures(u, v);
                            value = Math.Abs(gaussian) > 0 ? Math.Sqrt(Math.Abs(gaussian)) : Math.Abs(mean);
                        }
                        catch
                        {
                            // If even that fails (e.g., degenerate surface), assume flat
                            value = 0.0;
                        }
                    }

                    curvatureCache[key] = value;
                    return value;
                }

                // Sample curvature at center and edge midpoints
                double maxCurvatureCenter = GetMaxCurvatureCached(um, vm);
                double maxCurvatureU0 = GetMaxCurvatureCached(um, v0);
                double maxCurvatureU1 = GetMaxCurvatureCached(um, v1);
                double maxCurvatureV0 = GetMaxCurvatureCached(u0, vm);
                double maxCurvatureV1 = GetMaxCurvatureCached(u1, vm);

                double maxCurvature = Math.Max(maxCurvatureCenter,
                    Math.Max(Math.Max(maxCurvatureU0, maxCurvatureU1),
                             Math.Max(maxCurvatureV0, maxCurvatureV1)));

                // Subdivide if curvature exceeds tolerance
                if (maxCurvature > tolerance && depth < maxDepth)
                {
                    // subdivide into 4 quads
                    Subdivide(u0, um, v0, vm, depth + 1);
                    Subdivide(um, u1, v0, vm, depth + 1);
                    Subdivide(u0, um, vm, v1, depth + 1);
                    Subdivide(um, u1, vm, v1, depth + 1);
                }
                else
                {
                    // triangulate the quad into two triangles
                    var a = (u: u0, v: v0);
                    var b = (u: u1, v: v0);
                    var c = (u: u1, v: v1);
                    var d = (u: u0, v: v1);
                    AddTriangleByParams(a, b, c);
                    AddTriangleByParams(a, c, d);
                }
            }

            Subdivide(uStart, uEnd, vStart, vEnd, 0);

            return new Mesh(vertices.ToArray(), indexes.ToArray());
        }

        private readonly struct ParamKey
        {
            public readonly long UBits;
            public readonly long VBits;

            public ParamKey(double u, double v)
            {
                UBits = BitConverter.DoubleToInt64Bits(u);
                VBits = BitConverter.DoubleToInt64Bits(v);
            }
        }

        private sealed class ParamKeyComparer : IEqualityComparer<ParamKey>
        {
            public static readonly ParamKeyComparer Instance = new ParamKeyComparer();

            public bool Equals(ParamKey x, ParamKey y)
            {
                return x.UBits == y.UBits && x.VBits == y.VBits;
            }

            public int GetHashCode(ParamKey obj)
            {
                unchecked
                {
                    int hash = (int)obj.UBits ^ (int)(obj.UBits >> 32);
                    hash = (hash * 397) ^ ((int)obj.VBits ^ (int)(obj.VBits >> 32));
                    return hash;
                }
            }
        }
    }
}
