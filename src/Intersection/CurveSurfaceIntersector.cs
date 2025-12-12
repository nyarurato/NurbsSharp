using System;
using System.Collections.Generic;
using System.Linq;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;

namespace NurbsSharp.Intersection
{
    // TODO: Optimization

    /// <summary>
    /// (en) Intersector for NURBS curve and surface using marching method and Newton-Raphson iteration.
    /// Implementation strategy: 
    /// 1. March along the curve to find intersection candidates
    /// 2. Detect local minima in distance function and threshold crossings
    /// 3. Refine candidates using Newton-Raphson with adaptive damping
    /// (ja) マーチング法とNewton-Raphson反復法を使用したNURBS曲線とサーフェスの交差判定。
    /// 実装戦略:
    /// 1. 曲線に沿ってマーチングし交点候補を検出
    /// 2. 距離関数の局所最小値と閾値交差を検出
    /// 3. 適応的減衰を用いたNewton-Raphson法で候補を精密化
    /// </summary>
    public static class CurveSurfaceIntersector
    {
        /// <summary>
        /// (en) Tolerance for intersection convergence
        /// (ja) 交点収束の許容誤差
        /// </summary>
        public const double Tolerance = 1e-6;

        /// <summary>
        /// (en) Maximum number of iterations for Newton-Raphson
        /// (ja) Newton-Raphson法の最大反復回数
        /// </summary>
        public const int MaxIterations = 50;

        /// <summary>
        /// (en) Number of marching steps along the curve
        /// (ja) 曲線に沿ったマーチングステップ数
        /// </summary>
        private const int MarchingSteps = 200;

        /// <summary>
        /// (en) Find all intersections between a NURBS curve and surface
        /// (ja) NURBS曲線とサーフェス間の全交点検索
        /// </summary>
        /// <param name="curve">Curve to intersect</param>
        /// <param name="surface">Surface to intersect</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>List of intersection points</returns>
        /// <remarks>
        /// Uses BVH (Bounding Volume Hierarchy) for efficient candidate detection.
        /// Automatically falls back to marching method if BVH finds no candidates.
        /// </remarks>
        public static List<CurveSurfaceIntersection> Intersect(NurbsCurve curve, NurbsSurface surface, double tolerance = Tolerance)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            Guard.ThrowIfNull(surface, nameof(surface));

            // Build BVH for both curve and surface
            var curveBVH = CurveBVHBuilder.Build(curve);
            var surfaceBVH = SurfaceBVHBuilder.Build(surface);

            // Find intersection candidates using dual BVH
            var candidates = FindIntersectionCandidatesWithDualBVH(curve, surface, curveBVH, surfaceBVH, tolerance);

            // If no candidates found, fall back to standard marching method
            if (candidates.Count == 0)
            {
                candidates = FindIntersectionCandidatesUsingMarching(curve, surface, tolerance);
            }

            // Refine candidates using Newton-Raphson
            var intersections = new List<CurveSurfaceIntersection>();
            var processedCandidates = new HashSet<(double t, double u, double v)>();

            foreach (var candidate in candidates)
            {
                // Round to avoid near-duplicates
                var key = (Math.Round(candidate.t, 4), Math.Round(candidate.u, 4), Math.Round(candidate.v, 4));
                if (processedCandidates.Contains(key))
                    continue;

                processedCandidates.Add(key);

                if (TryRefineIntersection(curve, surface, candidate.t, candidate.u, candidate.v, tolerance,
                    out var intersection))
                {
                    // Check for duplicates
                    bool isDuplicate = false;
                    foreach (var existing in intersections)
                    {
                        if (Math.Abs(existing.U - intersection.U) < tolerance * 10 &&
                            Math.Abs(existing.SurfaceU - intersection.SurfaceU) < tolerance * 10 &&
                            Math.Abs(existing.SurfaceV - intersection.SurfaceV) < tolerance * 10)
                        {
                            isDuplicate = true;
                            break;
                        }
                    }

                    if (!isDuplicate)
                    {
                        intersections.Add(intersection);
                    }
                }
            }

            return intersections;
        }

        /// <summary>
        /// (en) Get intersecting leaf pairs between curve BVH and surface BVH using pair traversal
        /// (ja) カーブBVHとサーフェスBVHの葉ノードペアを再帰的にトラバースして取得
        /// </summary>
        private static List<(CurveBVHNode, SurfaceBVHNode)> GetIntersectingLeafPairs(CurveBVHNode curveRoot, SurfaceBVHNode surfaceRoot)
        {
            var result = new List<(CurveBVHNode, SurfaceBVHNode)>();

            void Traverse(CurveBVHNode a, SurfaceBVHNode b)
            {
                if (!a.Bounds.Intersects(b.Bounds)) return;
                if (a.IsLeaf && b.IsLeaf)
                {
                    result.Add((a, b));
                    return;
                }

                if (a.IsLeaf)
                {
                    if (b.Left != null) Traverse(a, b.Left);
                    if (b.Right != null) Traverse(a, b.Right);
                }
                else if (b.IsLeaf)
                {
                    if (a.Left != null) Traverse(a.Left, b);
                    if (a.Right != null) Traverse(a.Right, b);
                }
                else
                {
                    if (a.Left != null && b.Left != null) Traverse(a.Left, b.Left);
                    if (a.Left != null && b.Right != null) Traverse(a.Left, b.Right);
                    if (a.Right != null && b.Left != null) Traverse(a.Right, b.Left);
                    if (a.Right != null && b.Right != null) Traverse(a.Right, b.Right);
                }
            }

            Traverse(curveRoot, surfaceRoot);
            return result;
        }

        /// <summary>
        /// (en) Find intersection candidates using dual BVH acceleration (curve and surface)
        /// (ja) Dual BVH加速(カーブとサーフェス)を使用して交点候補を検索
        /// </summary>
        private static List<(double t, double u, double v, double dist)> FindIntersectionCandidatesWithDualBVH(
            NurbsCurve curve, NurbsSurface surface, CurveBVHNode curveBVH, SurfaceBVHNode surfaceBVH, double tolerance)
        {
            // Find all curve-surface region pairs that intersect using BVH pair traversal (more efficient)
            var candidateRegionPairs = GetIntersectingLeafPairs(curveBVH, surfaceBVH);

            // If no intersecting pairs found, fallback to standard method
            if (candidateRegionPairs.Count == 0)
            {
                return FindIntersectionCandidatesUsingMarching(curve, surface, tolerance);
            }

            // Sample curve segments in intersecting regions
            
            // Use concurrent bag for thread-safe collection
            var candidatesBag = new ConcurrentBag<(double t, double u, double v, double dist)>();

            // For improved performance, process leaf pairs in parallel
            Parallel.ForEach(candidateRegionPairs, pair =>
            {
                var (curveRegion, surfaceRegion) = pair;
                // Sample along curve in this region
                const int samplesPerRegion = 20;
                double minT = curveRegion.TRange.min;
                double maxT = curveRegion.TRange.max;
                double stepSize = (maxT - minT) / samplesPerRegion;
                
                var samples = new List<(double t, double u, double v, double dist)>();

                for (int i = 0; i <= samplesPerRegion; i++)
                {
                    double t = minT + stepSize * i;
                    Vector3Double curvePoint = CurveEvaluator.Evaluate(curve, t);
                    // Quick reject: if curve point far from surface region bbox skip
                    var dBox = surfaceRegion.Bounds.DistanceTo(curvePoint);
                    if (dBox > Math.Max(tolerance * 500, 0.1))
                        continue;
                    // Find closest point on surface within this region
                    double uMid = (surfaceRegion.URange.min + surfaceRegion.URange.max) * 0.5;
                    double vMid = (surfaceRegion.VRange.min + surfaceRegion.VRange.max) * 0.5;

                    var (u, v, surfacePoint, dist) = FindClosestPointOnSurfaceInRegion(
                        surface, curvePoint, surfaceRegion, uMid, vMid, tolerance);

                    samples.Add((t, u, v, dist));
                }

                // Detect candidates from samples in this region
                for (int i = 0; i < samples.Count; i++)
                {
                    var current = samples[i];

                    // Criterion 1: Very close to surface
                    if (current.dist < tolerance * 50)
                    {
                        // Add to concurrent bag (dedup later)
                        candidatesBag.Add(current);
                        continue;
                    }

                    // Criterion 2: Local minimum
                    if (i > 0 && i < samples.Count - 1)
                    {
                        var prev = samples[i - 1];
                        var next = samples[i + 1];

                        if (current.dist < prev.dist && current.dist < next.dist)
                        {
                            if (current.dist < tolerance * 200)
                            {
                                // Add candidate (duplicates removed later)
                                candidatesBag.Add(current);
                                continue;
                            }
                        }
                    }
                }
            });

            // Deduplicate candidates from concurrent bag
            var candidatesArray = candidatesBag.ToArray();
            var candidates = candidatesArray.GroupBy(c => Math.Round(c.t, 6)).Select(g => g.OrderBy(x => x.dist).First()).ToList();
            return candidates;
        }

        /// <summary>
        /// (en) Get all leaf nodes from curve BVH tree
        /// (ja) カーブBVHツリーから全ての葉ノードを取得
        /// </summary>
        private static List<CurveBVHNode> GetAllCurveLeaves(CurveBVHNode node)
        {
            var leaves = new List<CurveBVHNode>();
            if (node.IsLeaf)
            {
                leaves.Add(node);
            }
            else
            {
                if (node.Left != null)
                    leaves.AddRange(GetAllCurveLeaves(node.Left));
                if (node.Right != null)
                    leaves.AddRange(GetAllCurveLeaves(node.Right));
            }
            return leaves;
        }

        /// <summary>
        /// (en) Get all leaf nodes from surface BVH tree
        /// (ja) サーフェスBVHツリーから全ての葉ノードを取得
        /// </summary>
        private static List<SurfaceBVHNode> GetAllSurfaceLeaves(SurfaceBVHNode node)
        {
            var leaves = new List<SurfaceBVHNode>();
            if (node.IsLeaf)
            {
                leaves.Add(node);
            }
            else
            {
                if (node.Left != null)
                    leaves.AddRange(GetAllSurfaceLeaves(node.Left));
                if (node.Right != null)
                    leaves.AddRange(GetAllSurfaceLeaves(node.Right));
            }
            return leaves;
        }

        /// <summary>
        /// (en) Find intersection candidates using BVH acceleration
        /// (ja) BVH加速を使用して交点候補を検索
        /// </summary>
        private static List<(double t, double u, double v, double dist)> FindIntersectionCandidatesWithBVH(
            NurbsCurve curve, NurbsSurface surface, SurfaceBVHNode bvhRoot, double tolerance)
        {
            // Get curve parameter range
            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];

            // 1st pass: Sample along curve and find closest points
            double stepSize = (maxT - minT) / MarchingSteps;
            var samples = new List<(double t, double u, double v, double dist)>();

            for (int i = 0; i <= MarchingSteps; i++)
            {
                double t = minT + stepSize * i;
                Vector3Double curvePoint = CurveEvaluator.Evaluate(curve, t);

                // Find closest point on surface using BVH
                var (u, v, surfacePoint, dist) = FindClosestPointOnSurfaceWithBVH(
                    surface, curvePoint, bvhRoot, tolerance);

                samples.Add((t, u, v, dist));
            }

            // 2nd pass: Detect candidates from samples
            var candidates = new List<(double t, double u, double v, double dist)>();
            for (int i = 0; i < samples.Count; i++)
            {
                var current = samples[i];

                // Criterion 1: Very close to surface
                if (current.dist < tolerance * 50)
                {
                    candidates.Add(current);
                    continue;
                }

                // Criterion 2: Local minimum
                if (i > 0 && i < samples.Count - 1)
                {
                    var prev = samples[i - 1];
                    var next = samples[i + 1];

                    if (current.dist < prev.dist && current.dist < next.dist)
                    {
                        if (current.dist < tolerance * 200)
                        {
                            candidates.Add(current);
                        }
                    }
                }
            }

            return candidates;
        }

        /// <summary>
        /// (en) Find closest point on surface using BVH acceleration
        /// (ja) BVH加速を使用してサーフェス上の最近接点を検索
        /// </summary>
        private static (double u, double v, Vector3Double point, double distance) FindClosestPointOnSurfaceWithBVH(
            NurbsSurface surface, Vector3Double target, SurfaceBVHNode bvhRoot, double tolerance)
        {
            // Use moderate grid search only when BVH is used (more complex cases)
            return Operation.SurfaceOperator.FindClosestPoint(surface, target, tolerance, gridDivisions: 3);
        }

        /// <summary>
        /// (en) Find the closest point on surface to a given 3D point (fast version with initial guess)
        /// (ja) 初期推定値を使用した高速な最近接点探索
        /// </summary>
        private static (double u, double v, Vector3Double point, double distance) FindClosestPointOnSurface(
            NurbsSurface surface, Vector3Double target, double initialU, double initialV, double tolerance)
        {
            // Use single initial point for speed (no grid search)
            return Operation.SurfaceOperator.FindClosestPoint(surface, target, initialU, initialV, tolerance);
        }

        /// <summary>
        /// (en) Find the closest point on surface using grid search (accurate but slower)
        /// (ja) グリッド探索を使用した最近接点探索（正確だが低速）
        /// </summary>
        private static (double u, double v, Vector3Double point, double distance) FindClosestPointOnSurfaceWithGridSearch(
            NurbsSurface surface, Vector3Double target, double tolerance, int gridDivisions = 3)
        {
            // Use grid search for better accuracy when initial guess is unknown
            return Operation.SurfaceOperator.FindClosestPoint(surface, target, tolerance, gridDivisions);
        }

        /// <summary>
        /// (en) Find closest point on surface within specific parameter region
        /// (ja) 特定のパラメータ領域内でサーフェス上の最近接点を検索
        /// </summary>
        private static (double u, double v, Vector3Double point, double distance) FindClosestPointOnSurfaceInRegion(
            NurbsSurface surface, Vector3Double target, SurfaceBVHNode region, 
            double initialU, double initialV, double tolerance)
        {
            // Use initial guess from region center for speed
            return Operation.SurfaceOperator.FindClosestPoint(surface, target, initialU, initialV, tolerance);
        }

        /// <summary>
        /// (en) Get all leaf nodes from BVH tree
        /// (ja) BVHツリーから全ての葉ノードを取得
        /// </summary>
        private static List<SurfaceBVHNode> GetAllLeaves(SurfaceBVHNode node)
        {
            var leaves = new List<SurfaceBVHNode>();
            if (node.IsLeaf)
            {
                leaves.Add(node);
            }
            else
            {
                if (node.Left != null)
                    leaves.AddRange(GetAllLeaves(node.Left));
                if (node.Right != null)
                    leaves.AddRange(GetAllLeaves(node.Right));
            }
            return leaves;
        }

        /// <summary>
        /// (en) Find intersection candidates using marching method along the curve
        /// (ja) 曲線に沿ったマーチング法で交点候補を検索
        /// </summary>
        private static List<(double t, double u, double v, double dist)> FindIntersectionCandidatesUsingMarching(NurbsCurve curve, NurbsSurface surface, double tolerance)
        {
            var candidates = new List<(double t, double u, double v, double dist)>();

            // Get parameter ranges
            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];
            double minU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            // First pass: forward sampling using continuity initial guess
            double stepSize = (maxT - minT) / MarchingSteps;
            var forward = new List<(double t, double u, double v, double dist)>();

            double prevU = (minU + maxU) * 0.5;
            double prevV = (minV + maxV) * 0.5;

            for (int i = 0; i <= MarchingSteps; i++)
            {
                double t = minT + stepSize * i;
                Vector3Double curvePoint = CurveEvaluator.Evaluate(curve, t);

                var (u, v, surfacePoint, dist) = FindClosestPointOnSurface(
                    surface, curvePoint, prevU, prevV, tolerance);

                forward.Add((t, u, v, dist));
                prevU = u;
                prevV = v;
            }

            // Second pass: reverse sampling to reduce basin-of-attraction dependency
            var reverse = new List<(double t, double u, double v, double dist)>();
            prevU = (minU + maxU) * 0.5;
            prevV = (minV + maxV) * 0.5;

            for (int i = 0; i <= MarchingSteps; i++)
            {
                double t = maxT - stepSize * i;
                Vector3Double curvePoint = CurveEvaluator.Evaluate(curve, t);

                var (u, v, surfacePoint, dist) = FindClosestPointOnSurface(
                    surface, curvePoint, prevU, prevV, tolerance);

                reverse.Add((t, u, v, dist));
                prevU = u;
                prevV = v;
            }

            // Merge forward and reverse: prefer lower distance for same t (rounded)
            var mergedDict = new Dictionary<double, (double t, double u, double v, double dist)>();
            Action<(double t, double u, double v, double dist)> addOrReplace = sample =>
            {
                double key = Math.Round(sample.t, 8);
                if (!mergedDict.TryGetValue(key, out var exist) || sample.dist < exist.dist)
                    mergedDict[key] = sample;
            };

            foreach (var s in forward) addOrReplace(s);
            foreach (var s in reverse) addOrReplace(s);

            var samples = mergedDict.Values.OrderBy(s => s.t).ToList();

            // Adaptive local resampling:
            // If adjacent samples show large distance change, sign change in derivative,
            // or large jump in (u,v), then resample the interval densely.
            var augmented = new List<(double t, double u, double v, double dist)>(samples);

            for (int i = 0; i < samples.Count - 1; i++)
            {
                var a = samples[i];
                var b = samples[i + 1];

                // criteria for refinement
                bool largeDistChange = Math.Abs(b.dist - a.dist) > Math.Max(1e-3, Math.Min(a.dist, b.dist) * 0.5);
                double du = Math.Abs(a.u - b.u);
                double dv = Math.Abs(a.v - b.v);
                bool largeUVJump = du > 0.1 * (maxU - minU) || dv > 0.1 * (maxV - minV);
                double d1 = (i > 0) ? (a.dist - samples[i - 1].dist) : 0.0;
                double d2 = b.dist - a.dist;
                bool signChange = d1 * d2 < 0;

                if (largeDistChange || largeUVJump || signChange)
                {
                    const int subSteps = 10;
                    for (int k = 1; k < subSteps; k++)
                    {
                        double tt = a.t + (b.t - a.t) * (k / (double)subSteps);
                        Vector3Double cp = CurveEvaluator.Evaluate(curve, tt);

                        // Use grid search for better chance to find correct basin
                        var (uu, vv, sp, d) = Operation.SurfaceOperator.FindClosestPoint(surface, cp, tolerance, gridDivisions: 3);

                        augmented.Add((tt, uu, vv, d));
                    }
                }
            }

            // Rebuild sample list sorted by t after augmentation
            samples = augmented.OrderBy(s => s.t).ToList();

            // Candidate detection with expanded criteria
            double globalMin = samples.Min(s => s.dist);

            for (int i = 0; i < samples.Count; i++)
            {
                var current = samples[i];

                // High-priority: very close
                if (current.dist < tolerance * 20)
                {
                    candidates.Add(current);
                    continue;
                }

                // Near global minimum
                if (current.dist < Math.Max(globalMin * 2.0, tolerance * 100))
                {
                    candidates.Add(current);
                    continue;
                }

                // Local minimum (3-point)
                if (i > 0 && i < samples.Count - 1)
                {
                    var prev = samples[i - 1];
                    var next = samples[i + 1];
                    if (current.dist < prev.dist && current.dist < next.dist)
                    {
                        candidates.Add(current);
                        continue;
                    }
                }

                // Large UV jump may indicate crossing different surface patch -> candidate
                if (i > 0)
                {
                    var prev = samples[i - 1];
                    double duJump = Math.Abs(current.u - prev.u);
                    double dvJump = Math.Abs(current.v - prev.v);
                    if (duJump > 0.05 * (maxU - minU) || dvJump > 0.05 * (maxV - minV))
                    {
                        candidates.Add(current);
                        continue;
                    }
                }

                // Distance drop detection
                if (i > 0)
                {
                    var prev = samples[i - 1];
                    if (prev.dist > current.dist * 1.5 && current.dist < Math.Max(tolerance * 200, globalMin * 50))
                    {
                        candidates.Add(current);
                        continue;
                    }
                }
            }

            // Final duplicate pruning: keep one sample per small t-interval / uv-cluster
            var final = new List<(double t, double u, double v, double dist)>();
            foreach (var c in candidates.OrderBy(c => c.dist))
            {
                bool dup = final.Any(f =>
                    Math.Abs(f.t - c.t) < stepSize * 0.5 ||
                    (Math.Abs(f.u - c.u) < 1e-4 && Math.Abs(f.v - c.v) < 1e-4) ||
                    (Math.Abs(f.t - c.t) < 1e-6 && Math.Abs(f.dist - c.dist) < tolerance * 10)
                );
                if (!dup) final.Add(c);
            }

            return final;
        }

        /// <summary>
        /// (en) Refine an intersection point using Newton-Raphson iteration 
        /// (ja) Newton-Raphson反復法を使用して交点を精密化
        /// Solves: F(u,v,t) = S(u,v) - C(t) = 0
        /// Using Jacobian: A = [Fu, Fv, -Ft] where Fu=∂S/∂u, Fv=∂S/∂v, Ft=∂C/∂t
        /// </summary>
        private static bool TryRefineIntersection(NurbsCurve curve, NurbsSurface surface, double initialT, double initialU, double initialV, double tolerance, out CurveSurfaceIntersection intersection)
        {
            intersection = default;

            double t = initialT;  // Curve parameter
            double u = initialU;  // Surface U parameter
            double v = initialV;  // Surface V parameter

            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];
            double minU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                // Clamp parameters to valid range
                t = Math.Max(minT, Math.Min(maxT, t));
                u = Math.Max(minU, Math.Min(maxU, u));
                v = Math.Max(minV, Math.Min(maxV, v));

                // Evaluate: F = S(u,v) - C(t)
                Vector3Double surfPt = SurfaceEvaluator.Evaluate(surface, u, v);
                Vector3Double curvePt = CurveEvaluator.Evaluate(curve, t);
                Vector3Double F = surfPt - curvePt;
                
                double dist = F.magnitude;

                // Check convergence
                if (dist < tolerance)
                {
                    intersection = new CurveSurfaceIntersection
                    {
                        U = t,
                        SurfaceU = u,
                        SurfaceV = v,
                        CurvePoint = curvePt,
                        SurfacePoint = surfPt,
                        Distance = dist
                    };
                    return true;
                }

                // Evaluate derivatives
                var surfDerivs = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                Vector3Double Fu = surfDerivs.u_deriv;  // ∂S/∂u
                Vector3Double Fv = surfDerivs.v_deriv;  // ∂S/∂v
                Vector3Double Ft = CurveEvaluator.EvaluateFirstDerivative(curve, t);  // ∂C/∂t

                // Build Jacobian matrix A = [Fu, Fv, -Ft]
                //     |Fu.x  Fv.x  -Ft.x|
                // A = |Fu.y  Fv.y  -Ft.y|
                //     |Fu.z  Fv.z  -Ft.z|
                double a11 = Fu.X, a12 = Fv.X, a13 = -Ft.X;
                double a21 = Fu.Y, a22 = Fv.Y, a23 = -Ft.Y;
                double a31 = Fu.Z, a32 = Fv.Z, a33 = -Ft.Z;

                // Calculate determinant
                double det = a11 * (a22 * a33 - a23 * a32) -
                             a12 * (a21 * a33 - a23 * a31) +
                             a13 * (a21 * a32 - a22 * a31);

                if (Math.Abs(det) < 1e-12)
                {
                    // Singular matrix - accept if close enough
                    if (dist < tolerance * 50)
                    {
                        intersection = new CurveSurfaceIntersection
                        {
                            U = t,
                            SurfaceU = u,
                            SurfaceV = v,
                            CurvePoint = curvePt,
                            SurfacePoint = surfPt,
                            Distance = dist
                        };
                        return true;
                    }
                    return false;
                }

                // Right-hand side should be -F (Newton step solves A * d = -F)
                double bx = -F.X;
                double by = -F.Y;
                double bz = -F.Z;

                // Solve A·d = -F using Cramer's rule
                // d = [du, dv, dt]
                double du = (bx * (a22 * a33 - a23 * a32) -
                            a12 * (by * a33 - a23 * bz) +
                            a13 * (by * a32 - a22 * bz)) / det;

                double dv = (a11 * (by * a33 - a23 * bz) -
                            bx * (a21 * a33 - a23 * a31) +
                            a13 * (a21 * bz - by * a31)) / det;

                double dt = (a11 * (a22 * bz - by * a32) -
                            a12 * (a21 * bz - by * a31) +
                            bx * (a21 * a32 - a22 * a31)) / det;

                // Check convergence on increments
                if (Math.Abs(du) <= tolerance && Math.Abs(dv) <= tolerance && Math.Abs(dt) <= tolerance)
                {
                    intersection = new CurveSurfaceIntersection
                    {
                        U = t,
                        SurfaceU = u,
                        SurfaceV = v,
                        CurvePoint = curvePt,
                        SurfacePoint = surfPt,
                        Distance = dist
                    };
                    return true;
                }

                // Adaptive damping: try full step, if residual increases then halve step up to several times
                bool accepted = false;
                double alpha = 1.0;
                for (int damp = 0; damp < 6; damp++)
                {
                    double uNew = u + alpha * du;
                    double vNew = v + alpha * dv;
                    double tNew = t + alpha * dt;

                    // Clamp tentative params
                    uNew = Math.Max(minU, Math.Min(maxU, uNew));
                    vNew = Math.Max(minV, Math.Min(maxV, vNew));
                    tNew = Math.Max(minT, Math.Min(maxT, tNew));

                    Vector3Double surfPtNew = SurfaceEvaluator.Evaluate(surface, uNew, vNew);
                    Vector3Double curvePtNew = CurveEvaluator.Evaluate(curve, tNew);
                    double newDist = (surfPtNew - curvePtNew).magnitude;

                    if (newDist < dist || newDist < tolerance * 10)
                    {
                        // accept step
                        u = uNew;
                        v = vNew;
                        t = tNew;
                        accepted = true;
                        break;
                    }

                    alpha *= 0.5;
                }

                if (!accepted)
                {
                    // Could not decrease residual -> assume failure for this candidate
                    return false;
                }

                // Optionally continue iterations until convergence
            }

            // Did not converge
            return false;
        }

        /// <summary>
        /// (en) Test if curve intersects surface (boolean check only)
        /// (ja) 曲線がサーフェスと交差するか判定(真偽値のみ)
        /// </summary>
        /// <param name="curve">Curve to test</param>
        /// <param name="surface">Surface to test</param>
        /// <param name="tolerance">Convergence tolerance</param>
        /// <returns>True if curve intersects surface</returns>
        public static bool Intersects(NurbsCurve curve, NurbsSurface surface, double tolerance = Tolerance)
        {
            var intersections = Intersect(curve, surface, tolerance);
            return intersections.Count > 0;
        }
    }

    /// <summary>
    /// (en) Result of curve-surface intersection
    /// (ja) 曲線-サーフェス交点の結果
    /// </summary>
    public struct CurveSurfaceIntersection
    {
        /// <summary>
        /// (en) Parameter on curve
        /// (ja) 曲線のパラメータ
        /// </summary>
        public double U { get; set; }

        /// <summary>
        /// (en) Parameter on surface (U direction)
        /// (ja) サーフェスのパラメータ（U方向）
        /// </summary>
        public double SurfaceU { get; set; }

        /// <summary>
        /// (en) Parameter on surface (V direction)
        /// (ja) サーフェスのパラメータ（V方向）
        /// </summary>
        public double SurfaceV { get; set; }

        /// <summary>
        /// (en) Intersection point on curve
        /// (ja) 曲線上の交点
        /// </summary>
        public Vector3Double CurvePoint { get; set; }

        /// <summary>
        /// (en) Intersection point on surface
        /// (ja) サーフェス上の交点
        /// </summary>
        public Vector3Double SurfacePoint { get; set; }

        /// <summary>
        /// (en) Distance between the two points (should be near zero for true intersection)
        /// (ja) 2点間の距離（真の交点では0に近い）
        /// </summary>
        public double Distance { get; set; }
    }
}
