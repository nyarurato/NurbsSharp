using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) BVH node for NURBS curve parameter space subdivision
    /// (ja) NURBSカーブのパラメータ空間細分化用BVHノード
    /// </summary>
    internal class CurveBVHNode
    {
        /// <summary>
        /// (en) Bounding box in 3D space
        /// (ja) 3D空間での境界ボックス
        /// </summary>
        public BoundingBox Bounds { get; }

        /// <summary>
        /// (en) Parameter range in T direction
        /// (ja) T方向のパラメータ範囲
        /// </summary>
        public (double min, double max) TRange { get; }

        /// <summary>
        /// (en) Left child node
        /// (ja) 左の子ノード
        /// </summary>
        public CurveBVHNode? Left { get; }

        /// <summary>
        /// (en) Right child node
        /// (ja) 右の子ノード
        /// </summary>
        public CurveBVHNode? Right { get; }

        /// <summary>
        /// (en) Whether this is a leaf node
        /// (ja) 葉ノードかどうか
        /// </summary>
        public bool IsLeaf => Left == null && Right == null;

        public CurveBVHNode(BoundingBox bounds, (double, double) tRange, 
            CurveBVHNode? left = null, CurveBVHNode? right = null)
        {
            Bounds = bounds;
            TRange = tRange;
            Left = left;
            Right = right;
        }
    }

    /// <summary>
    /// (en) Builder for NURBS curve BVH
    /// (ja) NURBSカーブBVH構築クラス
    /// </summary>
    internal static class CurveBVHBuilder
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
        /// (en) Build BVH for a NURBS curve
        /// (ja) NURBSカーブ用のBVHを構築
        /// </summary>
        public static CurveBVHNode Build(NurbsCurve curve)
        {
            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];

            return BuildRecursive(curve, minT, maxT, 0);
        }

        private static CurveBVHNode BuildRecursive(NurbsCurve curve, 
            double minT, double maxT, int depth)
        {
            // Compute bounding box for this parameter region
            var bounds = ComputeRegionBounds(curve, minT, maxT);

            // Stop subdivision if max depth reached or region is too small
            if (depth >= MaxDepth || (maxT - minT) < MinRangeSize)
            {
                return new CurveBVHNode(bounds, (minT, maxT));
            }

            // Split in the middle
            double midT = (minT + maxT) * 0.5;

            var left = BuildRecursive(curve, minT, midT, depth + 1);
            var right = BuildRecursive(curve, midT, maxT, depth + 1);

            // Union of child bounds
            var unionBounds = left.Bounds.Union(right.Bounds);

            return new CurveBVHNode(unionBounds, (minT, maxT), left, right);
        }

        private static BoundingBox ComputeRegionBounds(NurbsCurve curve, double minT, double maxT)
        {
            // Extract the curve segment using SplitOperator for exact subdivision
            var segmentCurve = ExtractCurveSegment(curve, minT, maxT);
            
            // Use control points to build bounding box (NURBS convex hull property)
            var points = new List<Vector3Double>();
            foreach (var cp in segmentCurve.ControlPoints)
            {
                points.Add(cp.Position);
            }

            // Also add sampled points to ensure tight coverage
            const int samples = 10;
            for (int i = 0; i <= samples; i++)
            {
                double t = minT + (maxT - minT) * i / samples;
                points.Add(CurveEvaluator.Evaluate(curve, t));
            }

            return BoundingBox.FromPoints(points);
        }

        /// <summary>
        /// (en) Extract curve segment using SplitOperator
        /// (ja) SplitOperatorを使用してカーブセグメントを抽出
        /// </summary>
        private static NurbsCurve ExtractCurveSegment(NurbsCurve curve, double minT, double maxT)
        {
            double curveMin = curve.KnotVector.Knots[curve.Degree];
            double curveMax = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];

            // If the range covers the entire curve, return as is
            if (LinAlg.ApproxEqual(minT, curveMin) && LinAlg.ApproxEqual(maxT, curveMax))
            {
                return curve;
            }

            NurbsCurve workingCurve = curve;

            // Split at maxT first (if needed)
            if (!LinAlg.ApproxEqual(maxT, curveMax))
            {
                var (left, _) = SplitOperator.SplitCurve(workingCurve, maxT);
                workingCurve = left;
            }

            // Then split at minT (if needed)
            if (!LinAlg.ApproxEqual(minT, curveMin))
            {
                var (_, right) = SplitOperator.SplitCurve(workingCurve, minT);
                workingCurve = right;
            }

            return workingCurve;
        }

        /// <summary>
        /// (en) Traverse BVH and collect leaf nodes that intersect with a bounding box
        /// (ja) BVHをトラバースして境界ボックスと交差する葉ノードを収集
        /// </summary>
        public static List<CurveBVHNode> GetIntersectingLeaves(CurveBVHNode root, BoundingBox queryBounds)
        {
            var result = new List<CurveBVHNode>();
            TraverseForIntersection(root, queryBounds, result);
            return result;
        }

        private static void TraverseForIntersection(CurveBVHNode node, BoundingBox queryBounds, 
            List<CurveBVHNode> result)
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
