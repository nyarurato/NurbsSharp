using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using NurbsSharp.Generation.Approximation;
using NurbsSharp.Generation.Interpolation;


namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Provides methods for computing intersections between NURBS surfaces and planes
    /// (ja) NURBSサーフェスと平面の交差計算を提供する
    /// </summary>
    public static class SurfacePlaneIntersector
    {
        
        /*
         * Plane-Surface Fast Intersection algorithm for CNC Machining using bisection method 
         * https://www.nature.com/articles/s41598-025-25765-z
         * limitations:
         * - Only ONE intersection point between the cross section curve and the plane
         * flow:
         * 1. fixed u or v direction, generate an array of curves (need to decide which direction to fix based on the surface shape)
         * 2. for each curve, do bisection method to find intersection point with the plane
         * 3. collect all intersection points, generate intersection curve as b-spline using thomas algorithm
         * 4. interpolate the intersection points to generate a smooth curve
         */

        /// <summary>
        /// (en) Fast plane-surface intersection using bisection method (optimized for CNC machining)
        /// (ja) 二分法を使用した高速平面-サーフェス交差(CNC加工用に最適化)
        /// </summary>
        /// <param name="surface">NURBS surface to intersect</param>
        /// <param name="plane">Plane to intersect with</param>
        /// <param name="tolerance">Intersection tolerance</param>
        /// <param name="numIsoCurves">Number of isocurves to sample (default: 50)</param>
        /// <param name="fixU">If true, fix U and vary V; if false, fix V and vary U</param>
        /// <returns>List of intersection curves</returns>
        /// <remarks>
        /// This method assumes there is at most ONE intersection point between each isocurve and the plane.
        /// If multiple intersections exist, only one will be found per isocurve.
        /// </remarks>
        public static List<NurbsCurve> IntersectFast(NurbsSurface surface, Plane plane, double tolerance = 1e-6, int numIsoCurves = 50, bool? fixU = null)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            // Quick rejection using bounding box: if every corner of bounding box is on one side
            var bbox = surface.BoundingBox;
            var corners = new Vector3Double[] {
                bbox.Min,
                new Vector3Double(bbox.Min.X, bbox.Min.Y, bbox.Max.Z),
                new Vector3Double(bbox.Min.X, bbox.Max.Y, bbox.Min.Z),
                new Vector3Double(bbox.Min.X, bbox.Max.Y, bbox.Max.Z),
                new Vector3Double(bbox.Max.X, bbox.Min.Y, bbox.Min.Z),
                new Vector3Double(bbox.Max.X, bbox.Min.Y, bbox.Max.Z),
                new Vector3Double(bbox.Max.X, bbox.Max.Y, bbox.Min.Z),
                bbox.Max
            };
            double minD = double.PositiveInfinity;
            double maxD = double.NegativeInfinity;
            foreach (var c in corners)
            {
                double d = plane.SignedDistanceTo(c);
                if (d < minD) minD = d;
                if (d > maxD) maxD = d;
            }
            if (minD > tolerance || maxD < -tolerance)
                return new List<NurbsCurve>();
            
            // Determine which direction to fix based on surface aspect ratio (if not specified)
            if (!fixU.HasValue)
            {
                double uRange = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)] - surface.KnotVectorU.Knots[surface.DegreeU];
                double vRange = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)] - surface.KnotVectorV.Knots[surface.DegreeV];
                
                // Fix the direction with larger range to get better sampling
                fixU = uRange > vRange;
            }
            
            var intersectionPoints = new List<Vector3Double>();
            
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];
            
            if (fixU.Value)
            {
                // Fix U, vary V - generate curves along V direction
                for (int i = 0; i < numIsoCurves; i++)
                {
                    double u = uMin + (uMax - uMin) * i / (numIsoCurves - 1);
                    var isoCurve = surface.GetIsoCurveU(u);
                    
                    // Find intersection point using bisection
                    var point = FindCurvePlaneIntersectionBisection(isoCurve, plane, tolerance);
                    if (point.HasValue)
                    {
                        intersectionPoints.Add(point.Value);
                    }
                }
            }
            else
            {
                // Fix V, vary U - generate curves along U direction
                for (int i = 0; i < numIsoCurves; i++)
                {
                    double v = vMin + (vMax - vMin) * i / (numIsoCurves - 1);
                    var isoCurve = surface.GetIsoCurveV(v);
                    
                    // Find intersection point using bisection
                    var point = FindCurvePlaneIntersectionBisection(isoCurve, plane, tolerance);
                    if (point.HasValue)
                    {
                        intersectionPoints.Add(point.Value);
                    }
                }
            }
            
            var result = new List<NurbsCurve>();
            
            if (intersectionPoints.Count >= 2)
            {
                try
                {
                    // Interpolate the intersection points to create smooth curve
                    int degree = Math.Min(3, intersectionPoints.Count - 1);
                    var intersectionCurve = GlobalInterpolator.InterpolateCurve(intersectionPoints.ToArray(), degree);
                    result.Add(intersectionCurve);
                }
                catch
                {
                    // Fallback to linear if interpolation fails
                    if (intersectionPoints.Count >= 2)
                    {
                        int degree = Math.Min(1, intersectionPoints.Count - 1);
                        var intersectionCurve = GlobalInterpolator.InterpolateCurve(intersectionPoints.ToArray(), degree);
                        result.Add(intersectionCurve);
                    }
                }
            }
            
            return result;
        }

        /// <summary>
        /// (en) Find single intersection point between curve and plane using bisection method
        /// (ja) 二分法で曲線と平面の単一交点を探索
        /// </summary>
        /// <remarks>
        /// This assumes there is at most ONE intersection point on the curve.
        /// If multiple intersections exist, only one will be found.
        /// </remarks>
        private static Vector3Double? FindCurvePlaneIntersectionBisection(NurbsCurve curve, Plane plane, double tolerance)
        {
            double tMin = curve.KnotVector.Knots[curve.Degree];
            double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
            
            // Sample along curve to find sign change
            int numSamples = 200; // Increased for better detection
            double prevT = tMin;
            var prevPoint = curve.GetPos(prevT);
            double prevDist = plane.SignedDistanceTo(prevPoint);
            
            // Check if start point is already on plane
            if (Math.Abs(prevDist) < tolerance)
            {
                return prevPoint;
            }
            
            for (int i = 1; i <= numSamples; i++)
            {
                double t = tMin + (tMax - tMin) * i / numSamples;
                var point = curve.GetPos(t);
                double dist = plane.SignedDistanceTo(point);
                
                // Check if current point is on plane
                if (Math.Abs(dist) < tolerance)
                {
                    return point;
                }
                
                // Sign change indicates crossing
                if (prevDist * dist < 0)
                {
                    // Bisection refinement
                    double t1 = prevT;
                    double t2 = t;
                    
                    for (int iter = 0; iter < 50; iter++) // Max 50 bisection iterations
                    {
                        double tMid = (t1 + t2) * 0.5;
                        var pMid = curve.GetPos(tMid);
                        double dMid = plane.SignedDistanceTo(pMid);
                        
                        if (Math.Abs(dMid) < tolerance)
                        {
                            return pMid;
                        }
                        
                        var p1 = curve.GetPos(t1);
                        double d1 = plane.SignedDistanceTo(p1);
                        
                        if (d1 * dMid < 0)
                        {
                            t2 = tMid;
                        }
                        else
                        {
                            t1 = tMid;
                        }
                        
                        // Check convergence
                        if (Math.Abs(t2 - t1) < 1e-10)
                        {
                            break;
                        }
                    }
                    
                    // Return midpoint
                    double tFinal = (t1 + t2) * 0.5;
                    return curve.GetPos(tFinal);
                }
                
                prevT = t;
                prevPoint = point;
                prevDist = dist;
            }
            
            // No intersection found
            return null;
        }

        /// <summary>
        /// (en) Robust plane-surface intersection using marching/tracing method
        /// (ja) マーチング法(交点追跡法)を使用したロバストな平面-サーフェス交差
        /// </summary>
        /// <param name="surface">NURBS surface to intersect</param>
        /// <param name="plane">Plane to intersect with</param>
        /// <param name="tolerance">Intersection tolerance</param>
        /// <param name="maxIterations">Maximum Newton-Raphson iterations per point</param>
        /// <param name="stepSize">Initial step size for marching along intersection curve</param>
        /// <returns>List of intersection curves</returns>
        public static List<NurbsCurve> IntersectRobust(NurbsSurface surface, Plane plane, double tolerance = 1e-6, int maxIterations = 20, double stepSize = 0.01)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            // Quick rejection using bounding box
            var bbox = surface.BoundingBox;
            var corners = new Vector3Double[] {
                bbox.Min,
                new Vector3Double(bbox.Min.X, bbox.Min.Y, bbox.Max.Z),
                new Vector3Double(bbox.Min.X, bbox.Max.Y, bbox.Min.Z),
                new Vector3Double(bbox.Min.X, bbox.Max.Y, bbox.Max.Z),
                new Vector3Double(bbox.Max.X, bbox.Min.Y, bbox.Min.Z),
                new Vector3Double(bbox.Max.X, bbox.Min.Y, bbox.Max.Z),
                new Vector3Double(bbox.Max.X, bbox.Max.Y, bbox.Min.Z),
                bbox.Max
            };
            double minD = double.PositiveInfinity;
            double maxD = double.NegativeInfinity;
            foreach (var c in corners)
            {
                double d = plane.SignedDistanceTo(c);
                if (d < minD) minD = d;
                if (d > maxD) maxD = d;
            }
            if (minD > tolerance || maxD < -tolerance)
                return new List<NurbsCurve>();
            
            var result = new List<NurbsCurve>();
            
            // Find initial intersection points on surface boundary
            var seedPoints = FindBoundarySeedPoints(surface, plane, tolerance);
            
            if (seedPoints.Count == 0)
                return result; // No intersection found
            
            var visitedPoints = new HashSet<(double u, double v)>();
            
            // Trace each intersection curve from seed points
            foreach (var seed in seedPoints)
            {
                var key = (Math.Round(seed.u / tolerance) * tolerance, Math.Round(seed.v / tolerance) * tolerance);
                if (visitedPoints.Contains(key))
                    continue;
                
                var curvePoints = TraceCurve(surface, plane, seed, tolerance, maxIterations, stepSize, visitedPoints);
                
                if (curvePoints.Count >= 2)
                {
                    // Create NURBS curve from traced points using interpolation
                    var curve3DPoints = curvePoints.Select(p => surface.GetPos(p.u, p.v)).ToList();
                    
                    if (curve3DPoints.Count >= 2)
                    {
                        try
                        {
                            int degree = Math.Min(3, curve3DPoints.Count - 1);
                            var intersectionCurve = GlobalInterpolator.InterpolateCurve(curve3DPoints.ToArray(), degree);
                            result.Add(intersectionCurve);
                        }
                        catch
                        {
                            // Fallback to linear if interpolation fails
                            int degree = Math.Min(1, curve3DPoints.Count - 1);
                            var intersectionCurve = GlobalInterpolator.InterpolateCurve(curve3DPoints.ToArray(), degree);
                            result.Add(intersectionCurve);
                        }
                    }
                }
            }
            
            return result;
        }

        /// <summary>
        /// (en) Find seed points on surface boundary where plane intersects
        /// (ja) 平面がサーフェス境界と交差する初期点を探す
        /// </summary>
        private static List<(double u, double v)> FindBoundarySeedPoints(NurbsSurface surface, Plane plane, double tolerance)
        {
            var seeds = new List<(double u, double v)>();
            
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];
            
            int numSamples = 100; // Increased sampling for better detection
            
            // Check u-boundaries (v varies)
            for (int i = 0; i < numSamples; i++)
            {
                double v1 = vMin + i * (vMax - vMin) / numSamples;
                double v2 = vMin + (i + 1) * (vMax - vMin) / numSamples;
                v2 = Math.Min(v2, vMax); // Clamp to max
                
                // Check uMin boundary
                CheckBoundarySegment(surface, plane, uMin, v1, uMin, v2, 
                    tolerance, seeds, true);
                
                // Check uMax boundary
                CheckBoundarySegment(surface, plane, uMax, v1, uMax, v2, 
                    tolerance, seeds, true);
            }
            
            // Check v-boundaries (u varies)
            for (int i = 0; i < numSamples; i++)
            {
                double u1 = uMin + i * (uMax - uMin) / numSamples;
                double u2 = uMin + (i + 1) * (uMax - uMin) / numSamples;
                u2 = Math.Min(u2, uMax); // Clamp to max
                
                // Check vMin boundary
                CheckBoundarySegment(surface, plane, u1, vMin, u2, vMin, 
                    tolerance, seeds, false);
                
                // Check vMax boundary
                CheckBoundarySegment(surface, plane, u1, vMax, u2, vMax, 
                    tolerance, seeds, false);
            }
            
            // If no boundary seeds found, try interior grid sampling
            if (seeds.Count == 0)
            {
                int gridSize = 20;
                for (int i = 1; i < gridSize; i++)
                {
                    double u = uMin + i * (uMax - uMin) / gridSize;
                    for (int j = 1; j < gridSize; j++)
                    {
                        double v = vMin + j * (vMax - vMin) / gridSize;
                        var point = surface.GetPos(u, v);
                        double dist = Math.Abs(plane.SignedDistanceTo(point));
                        
                        if (dist < tolerance * 10)
                        {
                            // Refine with Newton-Raphson
                            var refined = RefineIntersectionPoint(surface, plane, u, v, tolerance, 10);
                            if (refined.HasValue)
                            {
                                seeds.Add(refined.Value);
                                // Found one seed, enough to start marching
                                return seeds;
                            }
                        }
                    }
                }
            }
            
            return seeds;
        }

        /// <summary>
        /// (en) Check if a boundary segment crosses the plane
        /// (ja) 境界セグメントが平面を横切るかチェック
        /// </summary>
        private static void CheckBoundarySegment(NurbsSurface surface, Plane plane,
            double u1, double v1, double u2, double v2, double tolerance,
            List<(double u, double v)> seeds, bool isUConstant)
        {
            var p1 = surface.GetPos(u1, v1);
            var p2 = surface.GetPos(u2, v2);
            
            double dist1 = plane.SignedDistanceTo(p1);
            double dist2 = plane.SignedDistanceTo(p2);
            // If an endpoint lies on the plane within tolerance, add it as a seed
            if (Math.Abs(dist1) <= tolerance)
            {
                if (!seeds.Any(s => Math.Abs(s.u - u1) < tolerance && Math.Abs(s.v - v1) < tolerance))
                    seeds.Add((u1, v1));
                return;
            }
            if (Math.Abs(dist2) <= tolerance)
            {
                if (!seeds.Any(s => Math.Abs(s.u - u2) < tolerance && Math.Abs(s.v - v2) < tolerance))
                    seeds.Add((u2, v2));
                return;
            }
            
            // Check if segment crosses plane (sign change)
            if (dist1 * dist2 < 0)
            {
                // Binary search for intersection point
                double uMid = u1, vMid = v1;
                double t = Math.Abs(dist1) / (Math.Abs(dist1) + Math.Abs(dist2));
                
                if (isUConstant)
                {
                    vMid = v1 + t * (v2 - v1);
                    uMid = u1;
                }
                else
                {
                    uMid = u1 + t * (u2 - u1);
                    vMid = v1;
                }
                
                // Refine with Newton-Raphson
                var refined = RefineIntersectionPoint(surface, plane, uMid, vMid, tolerance, 10);
                if (refined.HasValue)
                {
                    if (!seeds.Any(s => Math.Abs(s.u - refined.Value.u) < tolerance && Math.Abs(s.v - refined.Value.v) < tolerance))
                        seeds.Add(refined.Value);
                }
            }
        }

        /// <summary>
        /// (en) Trace intersection curve from seed point using marching method
        /// (ja) マーチング法で初期点から交差曲線を追跡
        /// </summary>
        private static List<(double u, double v)> TraceCurve(NurbsSurface surface, Plane plane,
            (double u, double v) seed, double tolerance, int maxIterations, double stepSize,
            HashSet<(double u, double v)> visitedPoints)
        {
            var points = new List<(double u, double v)> { seed };
            
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];
            
            // March in both directions from seed
            for (int direction = -1; direction <= 1; direction += 2)
            {
                var current = seed;
                
                for (int step = 0; step < 1000; step++) // Max steps to prevent infinite loop
                {
                    var key = (Math.Round(current.u / tolerance) * tolerance, 
                               Math.Round(current.v / tolerance) * tolerance);
                    
                    if (visitedPoints.Contains(key))
                        break;
                    
                    visitedPoints.Add(key);
                    
                    // Get tangent direction along intersection curve
                    var tangent = GetIntersectionCurveTangent(surface, plane, current.u, current.v);
                    
                    if (tangent.magnitude < 1e-10)
                        break; // Singular point
                    
                    // Take step in tangent direction
                    double du = direction * stepSize * tangent.X;
                    double dv = direction * stepSize * tangent.Y;
                    
                    double uNext = current.u + du;
                    double vNext = current.v + dv;
                    
                    // Check bounds
                    if (uNext < uMin || uNext > uMax || vNext < vMin || vNext > vMax)
                        break;
                    
                    // Project back to plane intersection using Newton-Raphson
                    var refined = RefineIntersectionPoint(surface, plane, uNext, vNext, tolerance, maxIterations);
                    
                    if (!refined.HasValue)
                        break;
                    
                    if (direction == 1)
                        points.Add(refined.Value);
                    else
                        points.Insert(0, refined.Value);
                    
                    current = refined.Value;
                    
                    // Check if we've closed the loop
                    double distToSeed = Math.Sqrt(Math.Pow(current.u - seed.u, 2) + Math.Pow(current.v - seed.v, 2));
                    if (step > 10 && distToSeed < stepSize * 2)
                        break;
                }
            }
            
            return points;
        }

        /// <summary>
        /// (en) Get tangent direction of intersection curve in parameter space
        /// (ja) パラメータ空間における交差曲線の接線方向を取得
        /// </summary>
        private static Vector3Double GetIntersectionCurveTangent(NurbsSurface surface, Plane plane, double u, double v)
        {
            var deriv = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
            
            // Tangent to intersection curve is perpendicular to both plane normal and surface normal
            var surfaceNormal = Vector3Double.Cross(deriv.u_deriv, deriv.v_deriv);
            var curveTangent3D = Vector3Double.Cross(plane.Normal, surfaceNormal);
            
            if (curveTangent3D.magnitude < 1e-10)
                return Vector3Double.Zero;
            
            curveTangent3D = curveTangent3D.normalized;
            
            // Project 3D tangent to parameter space using surface Jacobian
            // T_3D = Su * du/dt + Sv * dv/dt
            // We want to find (du/dt, dv/dt) given T_3D
            
            // Using least squares: [Su Sv] * [du/dt; dv/dt] = T_3D
            double a11 = deriv.u_deriv.X * deriv.u_deriv.X + deriv.u_deriv.Y * deriv.u_deriv.Y + deriv.u_deriv.Z * deriv.u_deriv.Z;
            double a12 = deriv.u_deriv.X * deriv.v_deriv.X + deriv.u_deriv.Y * deriv.v_deriv.Y + deriv.u_deriv.Z * deriv.v_deriv.Z;
            double a22 = deriv.v_deriv.X * deriv.v_deriv.X + deriv.v_deriv.Y * deriv.v_deriv.Y + deriv.v_deriv.Z * deriv.v_deriv.Z;
            
            double b1 = deriv.u_deriv.X * curveTangent3D.X + deriv.u_deriv.Y * curveTangent3D.Y + deriv.u_deriv.Z * curveTangent3D.Z;
            double b2 = deriv.v_deriv.X * curveTangent3D.X + deriv.v_deriv.Y * curveTangent3D.Y + deriv.v_deriv.Z * curveTangent3D.Z;
            
            double det = a11 * a22 - a12 * a12;
            
            if (Math.Abs(det) < 1e-10)
                return Vector3Double.Zero;
            
            double du_dt = (a22 * b1 - a12 * b2) / det;
            double dv_dt = (-a12 * b1 + a11 * b2) / det;
            
            return new Vector3Double(du_dt, dv_dt, 0);
        }

        /// <summary>
        /// (en) Refine intersection point using Newton-Raphson iteration
        /// (ja) Newton-Raphson法で交点を精密化
        /// </summary>
        private static (double u, double v)? RefineIntersectionPoint(NurbsSurface surface, Plane plane,
            double u0, double v0, double tolerance, int maxIterations)
        {
            double u = u0;
            double v = v0;
            
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];
            
            for (int iter = 0; iter < maxIterations; iter++)
            {
                // Clamp to bounds
                u = Math.Clamp(u, uMin, uMax);
                v = Math.Clamp(v, vMin, vMax);
                
                var point = surface.GetPos(u, v);
                double f = plane.SignedDistanceTo(point);
                
                if (Math.Abs(f) < tolerance)
                    return (u, v);
                
                var deriv = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                
                // Gradient of f with respect to (u,v)
                double dfdu = plane.Normal.X * deriv.u_deriv.X + plane.Normal.Y * deriv.u_deriv.Y + plane.Normal.Z * deriv.u_deriv.Z;
                double dfdv = plane.Normal.X * deriv.v_deriv.X + plane.Normal.Y * deriv.v_deriv.Y + plane.Normal.Z * deriv.v_deriv.Z;
                
                double gradMag = Math.Sqrt(dfdu * dfdu + dfdv * dfdv);
                
                if (gradMag < 1e-10)
                    return null; // Gradient too small, cannot converge
                
                // Newton step: move in direction of steepest descent
                u -= f * dfdu / (gradMag * gradMag);
                v -= f * dfdv / (gradMag * gradMag);
                
                // Check bounds
                if (u < uMin - tolerance || u > uMax + tolerance || v < vMin - tolerance || v > vMax + tolerance)
                    return null;
            }
            
            return null; // Did not converge
        }
    }


}