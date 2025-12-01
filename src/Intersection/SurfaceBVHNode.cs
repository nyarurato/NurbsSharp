using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) BVH node for NURBS surface parameter space subdivision
    /// (ja) NURBSサーフェスのパラメータ空間細分化用BVHノード
    /// </summary>
    internal class SurfaceBVHNode
    {
        /// <summary>
        /// (en) Bounding box in 3D space
        /// (ja) 3D空間での境界ボックス
        /// </summary>
        public BoundingBox Bounds { get; }

        /// <summary>
        /// (en) Parameter range in U direction
        /// (ja) U方向のパラメータ範囲
        /// </summary>
        public (double min, double max) URange { get; }

        /// <summary>
        /// (en) Parameter range in V direction
        /// (ja) V方向のパラメータ範囲
        /// </summary>
        public (double min, double max) VRange { get; }

        /// <summary>
        /// (en) Left child node
        /// (ja) 左の子ノード
        /// </summary>
        public SurfaceBVHNode? Left { get; }

        /// <summary>
        /// (en) Right child node
        /// (ja) 右の子ノード
        /// </summary>
        public SurfaceBVHNode? Right { get; }

        /// <summary>
        /// (en) Whether this is a leaf node
        /// (ja) 葉ノードかどうか
        /// </summary>
        public bool IsLeaf => Left == null && Right == null;

        public SurfaceBVHNode(BoundingBox bounds, (double, double) uRange, (double, double) vRange, 
            SurfaceBVHNode? left = null, SurfaceBVHNode? right = null)
        {
            Bounds = bounds;
            URange = uRange;
            VRange = vRange;
            Left = left;
            Right = right;
        }
    }

    /// <summary>
    /// (en) Builder for NURBS surface BVH
    /// (ja) NURBSサーフェスBVH構築クラス
    /// </summary>
    internal static class SurfaceBVHBuilder
    {
        /// <summary>
        /// (en) Maximum subdivision depth
        /// (ja) 最大細分化深度
        /// </summary>
        private const int MaxDepth = 4;

        /// <summary>
        /// (en) Minimum parameter range size to subdivide
        /// (ja) 細分化する最小パラメータ範囲サイズ
        /// </summary>
        private const double MinRangeSize = 0.05;

        /// <summary>
        /// (en) Build BVH for a NURBS surface
        /// (ja) NURBSサーフェス用のBVHを構築
        /// </summary>
        public static SurfaceBVHNode Build(NurbsSurface surface)
        {
            double minU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            return BuildRecursive(surface, minU, maxU, minV, maxV, 0);
        }

        private static SurfaceBVHNode BuildRecursive(NurbsSurface surface, 
            double minU, double maxU, double minV, double maxV, int depth)
        {
            // Compute bounding box for this parameter region
            var bounds = ComputeRegionBounds(surface, minU, maxU, minV, maxV);

            // Stop subdivision if max depth reached or region is too small
            double uRange = maxU - minU;
            double vRange = maxV - minV;
            if (depth >= MaxDepth || uRange < MinRangeSize || vRange < MinRangeSize)
            {
                return new SurfaceBVHNode(bounds, (minU, maxU), (minV, maxV));
            }

            // Choose split axis (alternate or use larger dimension)
            bool splitU = uRange > vRange;
            
            if (splitU)
            {
                double midU = (minU + maxU) * 0.5;
                var left = BuildRecursive(surface, minU, midU, minV, maxV, depth + 1);
                var right = BuildRecursive(surface, midU, maxU, minV, maxV, depth + 1);
                var combinedBounds = left.Bounds.Union(right.Bounds);
                return new SurfaceBVHNode(combinedBounds, (minU, maxU), (minV, maxV), left, right);
            }
            else
            {
                double midV = (minV + maxV) * 0.5;
                var left = BuildRecursive(surface, minU, maxU, minV, midV, depth + 1);
                var right = BuildRecursive(surface, minU, maxU, midV, maxV, depth + 1);
                var combinedBounds = left.Bounds.Union(right.Bounds);
                return new SurfaceBVHNode(combinedBounds, (minU, maxU), (minV, maxV), left, right);
            }
        }

        private static BoundingBox ComputeRegionBounds(NurbsSurface surface, 
            double minU, double maxU, double minV, double maxV)
        {
            // Extract the surface patch using SplitOperator for exact subdivision
            var patchSurface = ExtractSurfacePatch(surface, minU, maxU, minV, maxV);
            
            // Use control points to build bounding box (NURBS convex hull property)
            var points = new List<Vector3Double>();
            for (int i = 0; i < patchSurface.ControlPoints.Length; i++)
            {
                for (int j = 0; j < patchSurface.ControlPoints[i].Length; j++)
                {
                    var cp = patchSurface.ControlPoints[i][j];
                    points.Add(cp.Position);
                }
            }

            // Also add sampled points to ensure tight coverage
            const int samples = 5;
            for (int i = 0; i <= samples; i++)
            {
                double u = minU + (maxU - minU) * i / samples;
                for (int j = 0; j <= samples; j++)
                {
                    double v = minV + (maxV - minV) * j / samples;
                    points.Add(SurfaceEvaluator.Evaluate(surface, u, v));
                }
            }

            return BoundingBox.FromPoints(points);
        }

        /// <summary>
        /// (en) Extract surface patch using SplitOperator
        /// (ja) SplitOperatorを使用してサーフェスパッチを抽出
        /// </summary>
        private static NurbsSurface ExtractSurfacePatch(NurbsSurface surface, 
            double minU, double maxU, double minV, double maxV)
        {
            double surfaceMinU = surface.KnotVectorU.Knots[surface.DegreeU];
            double surfaceMaxU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double surfaceMinV = surface.KnotVectorV.Knots[surface.DegreeV];
            double surfaceMaxV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            // If the range covers the entire surface, return as is
            if (LinAlg.ApproxEqual(minU, surfaceMinU) && LinAlg.ApproxEqual(maxU, surfaceMaxU) &&
                LinAlg.ApproxEqual(minV, surfaceMinV) && LinAlg.ApproxEqual(maxV, surfaceMaxV))
            {
                return surface;
            }

            NurbsSurface workingSurface = surface;

            // Split in U direction first
            if (!LinAlg.ApproxEqual(maxU, surfaceMaxU))
            {
                var (left, _) = SplitOperator.SplitSurface(workingSurface, maxU, SurfaceDirection.U);
                workingSurface = left;
            }
            if (!LinAlg.ApproxEqual(minU, surfaceMinU))
            {
                var (_, right) = SplitOperator.SplitSurface(workingSurface, minU, SurfaceDirection.U);
                workingSurface = right;
            }

            // Then split in V direction
            if (!LinAlg.ApproxEqual(maxV, surfaceMaxV))
            {
                var (left, _) = SplitOperator.SplitSurface(workingSurface, maxV, SurfaceDirection.V);
                workingSurface = left;
            }
            if (!LinAlg.ApproxEqual(minV, surfaceMinV))
            {
                var (_, right) = SplitOperator.SplitSurface(workingSurface, minV, SurfaceDirection.V);
                workingSurface = right;
            }

            return workingSurface;
        }

        /// <summary>
        /// (en) Traverse BVH and collect leaf nodes that intersect with a bounding box
        /// (ja) BVHをトラバースして境界ボックスと交差する葉ノードを収集
        /// </summary>
        public static List<SurfaceBVHNode> GetIntersectingLeaves(SurfaceBVHNode root, BoundingBox queryBounds)
        {
            var result = new List<SurfaceBVHNode>();
            TraverseForIntersection(root, queryBounds, result);
            return result;
        }

        private static void TraverseForIntersection(SurfaceBVHNode node, BoundingBox queryBounds, 
            List<SurfaceBVHNode> result)
        {
            if (!node.Bounds.Intersects(queryBounds))
                return;

            if (node.IsLeaf)
            {
                result.Add(node);
            }
            else
            {
                if (node.Left != null)
                    TraverseForIntersection(node.Left, queryBounds, result);
                if (node.Right != null)
                    TraverseForIntersection(node.Right, queryBounds, result);
            }
        }
    }
}
