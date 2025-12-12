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
        /// <param name="surface"></param>
        /// <param name="numPointsU"></param>
        /// <param name="numPointsV"></param>
        /// <returns></returns>
        public static Mesh Tessellate(NurbsSurface surface,int numPointsU,int numPointsV)
        {
            Vector3Double[] vertices = new Vector3Double[numPointsU * numPointsV];
            int[] indexes = new int[(numPointsU - 1) * (numPointsV - 1) * 6];
            double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
            double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
            double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

            //TODO: Parallelize these loop
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
        /// <param name="surface"></param>
        /// <param name="tolerance">(en)Maximum curvature threshold (1/radius). Subdivides when max principal curvature exceeds this value; (ja)最大曲率の閾値（1/半径）。主曲率がこの値を超えると細分化</param>
        /// <param name="maxDepth">(en)Maximum recursion depth for subdivision; (ja)細分化の最大深さ</param>
        /// <returns></returns>
        public static Mesh TessellateAdaptive(NurbsSurface surface, double tolerance, int maxDepth = 8)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            Guard.ThrowIfNegativeOrZero(tolerance, nameof(tolerance));

            double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
            double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
            double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

            var vertices = new List<Vector3Double>();
            var indexes = new List<int>();
            var indexMap = new Dictionary<string, int>();

            string GetKey(double u, double v) => $"{u:R},{v:R}";

            int AddVertex(double u, double v)
            {
                var key = GetKey(u, v);
                if (indexMap.TryGetValue(key, out var idx))
                    return idx;
                var p = surface.GetPos(u, v);
                idx = vertices.Count;
                vertices.Add(p);
                indexMap[key] = idx;
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

                // Helper to safely get max absolute curvature, returning 0 on failure
                double GetMaxCurvatureSafe(double u, double v)
                {
                    try
                    {
                        var (k1, k2) = surface.GetPrincipalCurvatures(u, v);
                        return Math.Max(Math.Abs(k1), Math.Abs(k2));
                    }
                    catch
                    {
                        // Fallback: use Gaussian curvature (product of principal curvatures)
                        try
                        {
                            var (mean, gaussian) = surface.GetMeanAndGaussianCurvatures(u, v);
                            return Math.Abs(gaussian) > 0 ? Math.Sqrt(Math.Abs(gaussian)) : Math.Abs(mean);
                        }
                        catch
                        {
                            // If even that fails (e.g., degenerate surface), assume flat
                            return 0.0;
                        }
                    }
                }

                // Sample curvature at center and edge midpoints
                double maxCurvatureCenter = GetMaxCurvatureSafe(um, vm);
                double maxCurvatureU0 = GetMaxCurvatureSafe(um, v0);
                double maxCurvatureU1 = GetMaxCurvatureSafe(um, v1);
                double maxCurvatureV0 = GetMaxCurvatureSafe(u0, vm);
                double maxCurvatureV1 = GetMaxCurvatureSafe(u1, vm);

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
    }
}
