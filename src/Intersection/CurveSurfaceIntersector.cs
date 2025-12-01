using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;

namespace NurbsSharp.Intersection
{
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
        /// (en) Find all intersections between a NURBS curve and surface using marching method and Newton-Raphson
        /// (ja) マーチング法とNewton-Raphson法を使用してNURBS曲線とサーフェス間の全交点を検索
        /// </summary>
        /// <param name="curve">Curve to intersect</param>
        /// <param name="surface">Surface to intersect</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>List of intersection points</returns>
        public static List<CurveSurfaceIntersection> Intersect(NurbsCurve curve, NurbsSurface surface, double tolerance = Tolerance)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            Guard.ThrowIfNull(surface, nameof(surface));

            var intersections = new List<CurveSurfaceIntersection>();

            // Early out: check bounding boxes
            if (!curve.BoundingBox.Intersects(surface.BoundingBox))
                return intersections;

            // Use marching method to find intersection candidates
            var candidates = FindIntersectionCandidatesUsingMarching(curve, surface, tolerance);

            // Refine each candidate using Newton-Raphson
            var refinedIntersections = new List<CurveSurfaceIntersection>();
            foreach (var candidate in candidates)
            {
                if (TryRefineIntersection(curve, surface, candidate.t, candidate.u, candidate.v,
                    tolerance, out CurveSurfaceIntersection intersection))
                {
                    // Check if this intersection is unique (not duplicate)
                    bool isDuplicate = false;
                    foreach (var existing in refinedIntersections)
                    {
                        double paramDist = Math.Abs(existing.U - intersection.U);
                        double spatialDist = (existing.CurvePoint - intersection.CurvePoint).magnitude;
                        
                        // More lenient spatial check, stricter parameter check
                        if (paramDist < tolerance * 5 || spatialDist < tolerance * 5)
                        {
                            isDuplicate = true;
                            break;
                        }
                    }

                    if (!isDuplicate)
                    {
                        refinedIntersections.Add(intersection);
                    }
                }
            }

            return refinedIntersections;
        }

        /// <summary>
        /// (en) Find all intersections using BVH acceleration for both curve and surface
        /// (ja) CurveとSurface両方のBVH加速を使用した交差判定
        /// </summary>
        /// <param name="curve">Curve to intersect</param>
        /// <param name="surface">Surface to intersect</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>List of intersection points</returns>
        public static List<CurveSurfaceIntersection> IntersectWithBVH(NurbsCurve curve, NurbsSurface surface, double tolerance = Tolerance)
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
                return Intersect(curve, surface, tolerance);
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
        /// (en) Find intersection candidates using dual BVH acceleration (curve and surface)
        /// (ja) Dual BVH加速(カーブとサーフェス)を使用して交点候補を検索
        /// </summary>
        private static List<(double t, double u, double v, double dist)> FindIntersectionCandidatesWithDualBVH(
            NurbsCurve curve, NurbsSurface surface, CurveBVHNode curveBVH, SurfaceBVHNode surfaceBVH, double tolerance)
        {
            // Get all curve and surface leaf nodes
            var curveLeaves = GetAllCurveLeaves(curveBVH);
            var surfaceLeaves = GetAllSurfaceLeaves(surfaceBVH);

            // Find all curve-surface region pairs that intersect
            var candidateRegionPairs = new List<(CurveBVHNode curveRegion, SurfaceBVHNode surfaceRegion)>();
            
            foreach (var curveRegion in curveLeaves)
            {
                foreach (var surfaceRegion in surfaceLeaves)
                {
                    if (curveRegion.Bounds.Intersects(surfaceRegion.Bounds))
                    {
                        candidateRegionPairs.Add((curveRegion, surfaceRegion));
                    }
                }
            }

            // If no intersecting pairs found, fallback to standard method
            if (candidateRegionPairs.Count == 0)
            {
                return FindIntersectionCandidatesUsingMarching(curve, surface, tolerance);
            }

            // Sample curve segments in intersecting regions
            var candidates = new List<(double t, double u, double v, double dist)>();
            
            foreach (var (curveRegion, surfaceRegion) in candidateRegionPairs)
            {
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
                        // Check if not duplicate
                        bool isDuplicate = false;
                        foreach (var existing in candidates)
                        {
                            if (Math.Abs(existing.t - current.t) < tolerance * 10)
                            {
                                isDuplicate = true;
                                break;
                            }
                        }
                        if (!isDuplicate)
                        {
                            candidates.Add(current);
                        }
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
                                // Check if not duplicate
                                bool isDuplicate = false;
                                foreach (var existing in candidates)
                                {
                                    if (Math.Abs(existing.t - current.t) < tolerance * 10)
                                    {
                                        isDuplicate = true;
                                        break;
                                    }
                                }
                                if (!isDuplicate)
                                {
                                    candidates.Add(current);
                                }
                            }
                        }
                    }
                }
            }

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
            // Search in all leaf nodes and find the best
            var leaves = GetAllLeaves(bvhRoot);
            
            double bestU = 0, bestV = 0;
            Vector3Double bestPoint = Vector3Double.Zero;
            double bestDist = double.MaxValue;

            foreach (var region in leaves)
            {
                double uMid = (region.URange.min + region.URange.max) * 0.5;
                double vMid = (region.VRange.min + region.VRange.max) * 0.5;

                var (u, v, point, dist) = FindClosestPointOnSurfaceInRegion(
                    surface, target, region, uMid, vMid, tolerance);

                if (dist < bestDist)
                {
                    bestU = u;
                    bestV = v;
                    bestPoint = point;
                    bestDist = dist;
                }
            }

            return (bestU, bestV, bestPoint, bestDist);
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
        /// (en) Find closest point on surface within a specific parameter region
        /// (ja) 特定のパラメータ領域内でサーフェス上の最近接点を検索
        /// </summary>
        private static (double u, double v, Vector3Double point, double distance) FindClosestPointOnSurfaceInRegion(
            NurbsSurface surface, Vector3Double target, SurfaceBVHNode region, 
            double initialU, double initialV, double tolerance)
        {
            // Use the standard closest point finder without region constraints
            // The region is only used for initial guess
            return FindClosestPointOnSurface(surface, target, initialU, initialV, tolerance);
        }

        /// <summary>
        /// (en) Find intersection candidates using marching method along the curve
        /// (ja) 曲線に沿ったマーチング法で交点候補を検索
        /// </summary>
        private static List<(double t, double u, double v, double dist)> FindIntersectionCandidatesUsingMarching(
            NurbsCurve curve, NurbsSurface surface, double tolerance)
        {
            var candidates = new List<(double t, double u, double v, double dist)>();

            // Get parameter ranges
            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];
            double minU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            // First pass: collect all distance samples
            double stepSize = (maxT - minT) / MarchingSteps;
            var samples = new List<(double t, double u, double v, double dist)>();
            (double u, double v) previousClosest = ((minU + maxU) / 2, (minV + maxV) / 2);

            for (int i = 0; i <= MarchingSteps; i++)
            {
                double t = minT + stepSize * i;
                Vector3Double curvePoint = CurveEvaluator.Evaluate(curve, t);

                // Find closest point on surface
                var (u, v, surfacePoint, dist) = FindClosestPointOnSurface(
                    surface, curvePoint, previousClosest.u, previousClosest.v, tolerance);

                samples.Add((t, u, v, dist));
                previousClosest = (u, v);
            }

            // Second pass: detect candidates
            for (int i = 0; i < samples.Count; i++)
            {
                var current = samples[i];

                // Criterion 1: Very close to surface
                if (current.dist < tolerance * 50)
                {
                    candidates.Add(current);
                    continue;
                }

                // Criterion 2: Local minimum (3-point check)
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

                // Criterion 3: Distance drop detection
                if (i > 0)
                {
                    var prev = samples[i - 1];
                    if (prev.dist > tolerance * 100 && current.dist < tolerance * 100)
                    {
                        candidates.Add(prev);
                        candidates.Add(current);
                    }
                }
            }

            return candidates;
        }

        /// <summary>
        /// (en) Find the closest point on surface to a given 3D point using Newton-Raphson
        /// (ja) Newton-Raphson法で指定された3D点に最も近いサーフェス上の点を検索
        /// </summary>
        private static (double u, double v, Vector3Double point, double distance) FindClosestPointOnSurface(
            NurbsSurface surface, Vector3Double target, double initialU, double initialV, double tolerance)
        {
            double minU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            double u = Math.Max(minU, Math.Min(maxU, initialU));
            double v = Math.Max(minV, Math.Min(maxV, initialV));

            for (int iter = 0; iter < 30; iter++) // More iterations for better accuracy
            {
                Vector3Double sp = SurfaceEvaluator.Evaluate(surface, u, v);
                var derivs = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                Vector3Double surfDU = derivs.u_deriv;
                Vector3Double surfDV = derivs.v_deriv;

                Vector3Double delta = sp - target;
                double dist = delta.magnitude;

                if (dist < tolerance || iter > 15)
                {
                    return (u, v, sp, dist);
                }

                // Minimize ||S(u,v) - target||^2
                // Gradient: g = [2*delta·surfDU, 2*delta·surfDV]
                double g1 = 2 * Vector3Double.Dot(delta, surfDU);
                double g2 = 2 * Vector3Double.Dot(delta, surfDV);

                // Hessian approximation (ignore second derivatives)
                double h11 = 2 * Vector3Double.Dot(surfDU, surfDU);
                double h12 = 2 * Vector3Double.Dot(surfDU, surfDV);
                double h22 = 2 * Vector3Double.Dot(surfDV, surfDV);

                double det = h11 * h22 - h12 * h12;
                if (Math.Abs(det) < 1e-12)
                    break;

                double deltaU = -(h22 * g1 - h12 * g2) / det;
                double deltaV = -(-h12 * g1 + h11 * g2) / det;

                // Apply damping
                double damping = 0.7;
                u += damping * deltaU;
                v += damping * deltaV;

                // Clamp to bounds
                u = Math.Max(minU, Math.Min(maxU, u));
                v = Math.Max(minV, Math.Min(maxV, v));
            }

            Vector3Double finalPoint = SurfaceEvaluator.Evaluate(surface, u, v);
            double finalDist = (finalPoint - target).magnitude;
            return (u, v, finalPoint, finalDist);
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

                // Solve A·d = F using Cramer's rule
                // d = [du, dv, dt]
                double du = (F.X * (a22 * a33 - a23 * a32) -
                            a12 * (F.Y * a33 - a23 * F.Z) +
                            a13 * (F.Y * a32 - a22 * F.Z)) / det;

                double dv = (a11 * (F.Y * a33 - a23 * F.Z) -
                            F.X * (a21 * a33 - a23 * a31) +
                            a13 * (a21 * F.Z - F.Y * a31)) / det;

                double dt = (a11 * (a22 * F.Z - F.Y * a32) -
                            a12 * (a21 * F.Z - F.Y * a31) +
                            F.X * (a21 * a32 - a22 * a31)) / det;

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

                // Update parameters
                u += du;
                v += dv;
                t += dt;

                // Check if parameters went out of bounds
                if (t < minT || t > maxT || u < minU || u > maxU || v < minV || v > maxV)
                {
                    return false;
                }
            }

            // Did not converge
            return false;
        }

        /// <summary>
        /// (en) Check if a curve intersects a surface (boolean only)
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
