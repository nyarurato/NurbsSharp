using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
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
        /// https://www.nature.com/articles/s41598-025-25765-z
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
            
            // Prepare parameter bounds
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];

            // Determine which direction to fix when not specified. Use a combined heuristic:
            // 1) Compare representative surface-direction vectors (how much moving along an isocurve
            //    changes the point relative to plane normal). Prefer fixing U (i.e., sampling V)
            //    when the V-direction vector has larger projection on the plane normal.
            // 2) Fall back to parameter-range heuristic when the vector scores are similar.
            if (!fixU.HasValue)
            {
                double uRange = uMax - uMin;
                double vRange = vMax - vMin;

                double uMid = 0.5 * (uMin + uMax);
                double vMid = 0.5 * (vMin + vMax);

                // Representative 3D direction vectors for iso-curves
                var pU1 = surface.GetPos(uMin, vMid);
                var pU2 = surface.GetPos(uMax, vMid);
                var pV1 = surface.GetPos(uMid, vMin);
                var pV2 = surface.GetPos(uMid, vMax);

                var su = pU2 - pU1; // direction when varying U (fix V)
                var sv = pV2 - pV1; // direction when varying V (fix U)

                double suNorm = su.magnitude;
                double svNorm = sv.magnitude;

                double suScore = 0.0;
                double svScore = 0.0;

                if (suNorm > 1e-12)
                {
                    suScore = Math.Abs(su.X * plane.Normal.X + su.Y * plane.Normal.Y + su.Z * plane.Normal.Z) / suNorm;
                }
                if (svNorm > 1e-12)
                {
                    svScore = Math.Abs(sv.X * plane.Normal.X + sv.Y * plane.Normal.Y + sv.Z * plane.Normal.Z) / svNorm;
                }

                // If scores differ significantly, prefer the vector-based choice.
                if (Math.Abs(svScore - suScore) > 0.05)
                {
                    // fixU == true means we fix U and vary V (i.e., iso-curves run along V -> sv)
                    fixU = svScore > suScore;
                }
                else
                {
                    // Fallback to simple parameter-range heuristic
                    fixU = uRange > vRange;
                }
            }

            var intersectionPoints = new List<Vector3Double>();
            
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
            
            // Find initial intersection points (boundary + interior)
            var seedPoints = FindBoundarySeedPoints(surface, plane, tolerance);
            
            // Filter out degenerate seeds and add interior seeds
            var validSeeds = new List<(double u, double v)>();
            foreach (var seed in seedPoints)
            {
                var tangent = GetIntersectionCurveTangent(surface, plane, seed.u, seed.v);
                if (tangent.magnitude > 1e-8)
                {
                    validSeeds.Add(seed);
                }
            }
            
            // Always try interior sampling to find additional seeds
            var interiorSeeds = FindInteriorSeedPoints(surface, plane, tolerance);
            foreach (var seed in interiorSeeds)
            {
                // Check if not duplicate
                bool isDuplicate = false;
                foreach (var existing in validSeeds)
                {
                    double dist = Math.Sqrt(Math.Pow(existing.u - seed.u, 2) + Math.Pow(existing.v - seed.v, 2));
                    if (dist < tolerance * 100)
                    {
                        isDuplicate = true;
                        break;
                    }
                }
                if (!isDuplicate)
                {
                    validSeeds.Add(seed);
                }
            }
            
            if (validSeeds.Count == 0)
                return result;
            
            // Track which seeds have been visited
            var visitedSeeds = new bool[validSeeds.Count];
            var allVisited = false;
            
            // Trace curves from each unvisited seed
            int seedIndex = 0;
            while (!allVisited && seedIndex < validSeeds.Count)
            {
                if (visitedSeeds[seedIndex])
                {
                    seedIndex++;
                    continue;
                }
                
                // Trace curve in both directions from this seed
                var curvePoints = TraceCurveBidirectional(surface, plane, validSeeds[seedIndex], 
                    tolerance, maxIterations, stepSize, validSeeds, visitedSeeds, seedIndex);
                
                if (curvePoints.Count >= 2)
                {
                    var curve3DPoints = curvePoints.Select(p => surface.GetPos(p.u, p.v)).ToList();
                    
                    // Remove duplicate points
                    var uniquePoints = new List<Vector3Double>();
                    foreach (var pt in curve3DPoints)
                    {
                        bool isDup = false;
                        foreach (var existing in uniquePoints)
                        {
                            if ((pt - existing).magnitude < tolerance * 10)
                            {
                                isDup = true;
                                break;
                            }
                        }
                        if (!isDup) uniquePoints.Add(pt);
                    }
                    
                    if (uniquePoints.Count >= 2)
                    {
                        try
                        {
                            int degree = Math.Min(3, uniquePoints.Count - 1);
                            var curve = GlobalInterpolator.InterpolateCurve(uniquePoints.ToArray(), degree);
                            result.Add(curve);
                        }
                        catch
                        {
                            try
                            {
                                int degree = Math.Min(1, uniquePoints.Count - 1);
                                var curve = GlobalInterpolator.InterpolateCurve(uniquePoints.ToArray(), degree);
                                result.Add(curve);
                            }
                            catch { }
                        }
                    }
                }
                
                seedIndex++;
                
                // Check if all seeds visited
                allVisited = true;
                for (int i = 0; i < visitedSeeds.Length; i++)
                {
                    if (!visitedSeeds[i])
                    {
                        allVisited = false;
                        seedIndex = i;
                        break;
                    }
                }
            }
            
            return result;
        }
        
        /// <summary>
        /// (en) Trace curve bidirectionally from seed point and mark visited seeds
        /// (ja) 初期点から双方向にカーブをトレースし、訪問済み初期点をマーク
        /// </summary>
        private static List<(double u, double v)> TraceCurveBidirectional(NurbsSurface surface, Plane plane, 
            (double u, double v) seed, double tolerance, int maxIterations, double stepSize,
            List<(double u, double v)> allSeeds, bool[] visitedSeeds, int currentSeedIndex)
        {
            var allPoints = new List<(double u, double v)>();
            
            // Mark current seed as visited
            visitedSeeds[currentSeedIndex] = true;
            
            // Trace forward direction
            var forwardPoints = TraceCurveDirection(surface, plane, seed, 1, tolerance, maxIterations, stepSize, allSeeds, visitedSeeds);
            
            // Trace inverse direction
            var inversePoints = TraceCurveDirection(surface, plane, seed, -1, tolerance, maxIterations, stepSize, allSeeds, visitedSeeds);
            
            // Combine: inverse (reversed) + seed + forward
            for (int i = inversePoints.Count - 1; i >= 0; i--)
            {
                allPoints.Add(inversePoints[i]);
            }
            allPoints.Add(seed);
            allPoints.AddRange(forwardPoints);
            
            return allPoints;
        }
        
        /// <summary>
        /// (en) Trace curve in one direction from seed point
        /// (ja) 初期点から指定方向にカーブをトレース
        /// </summary>
        private static List<(double u, double v)> TraceCurveDirection(NurbsSurface surface, Plane plane, 
            (double u, double v) seed, int direction, double tolerance, int maxIterations, double stepSize,
            List<(double u, double v)> allSeeds, bool[] visitedSeeds)
        {
            var points = new List<(double u, double v)>();
            
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];
            
            double uCurrent = seed.u;
            double vCurrent = seed.v;
            int consecutiveOutOfBoundsCount = 0;
            
            for (int step = 0; step < 10000; step++)
            {
                var tangent = GetIntersectionCurveTangent(surface, plane, uCurrent, vCurrent);
                
                if (tangent.magnitude < 1e-10)
                    break;
                
                tangent = tangent.normalized;
                
                // Step in specified direction
                double uNext = uCurrent + direction * stepSize * tangent.X;
                double vNext = vCurrent + direction * stepSize * tangent.Y;
                
                // Check if out of bounds
                if (uNext < uMin || uNext > uMax || vNext < vMin || vNext > vMax)
                {
                    consecutiveOutOfBoundsCount++;
                    
                    // Clamp to bounds
                    uNext = Math.Max(uMin, Math.Min(uMax, uNext));
                    vNext = Math.Max(vMin, Math.Min(vMax, vNext));
                    
                    // Break if consistently out of bounds
                    if (consecutiveOutOfBoundsCount > 5)
                        break;
                }
                else
                {
                    consecutiveOutOfBoundsCount = 0;
                }
                
                // Newton-Raphson refinement
                for (int iter = 0; iter < maxIterations; iter++)
                {
                    var pt = surface.GetPos(uNext, vNext);
                    double dist = plane.SignedDistanceTo(pt);
                    
                    if (Math.Abs(dist) < tolerance)
                        break;
                    
                    var deriv = SurfaceEvaluator.EvaluateFirstDerivative(surface, uNext, vNext);
                    
                    // Gradient of distance with respect to (u,v)
                    double dfdu = plane.Normal.X * deriv.u_deriv.X + plane.Normal.Y * deriv.u_deriv.Y + plane.Normal.Z * deriv.u_deriv.Z;
                    double dfdv = plane.Normal.X * deriv.v_deriv.X + plane.Normal.Y * deriv.v_deriv.Y + plane.Normal.Z * deriv.v_deriv.Z;
                    
                    double gradMag = Math.Sqrt(dfdu * dfdu + dfdv * dfdv);
                    
                    if (gradMag < 1e-10)
                        break;
                    
                    uNext -= dist * dfdu / (gradMag * gradMag);
                    vNext -= dist * dfdv / (gradMag * gradMag);
                }
                
                points.Add((uNext, vNext));
                
                // Check if reached another seed (loop closure)
                for (int i = 0; i < allSeeds.Count; i++)
                {
                    if (visitedSeeds[i])
                        continue;
                        
                    double dist = Math.Sqrt(Math.Pow(allSeeds[i].u - uNext, 2) + Math.Pow(allSeeds[i].v - vNext, 2));
                    if (dist < stepSize * 0.5)
                    {
                        visitedSeeds[i] = true;
                    }
                }
                
                // Check loop closure to original seed
                if (step > 20)
                {
                    double distToSeed = Math.Sqrt(Math.Pow(seed.u - uNext, 2) + Math.Pow(seed.v - vNext, 2));
                    if (distToSeed < stepSize * 0.5)
                        break;
                }
                
                uCurrent = uNext;
                vCurrent = vNext;
            }
            
            return points;
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
        /// (en) Find seed points in surface interior where plane intersects
        /// (ja) サーフェス内部で平面が交差する初期点を探す
        /// </summary>
        private static List<(double u, double v)> FindInteriorSeedPoints(NurbsSurface surface, Plane plane, double tolerance)
        {
            var seeds = new List<(double u, double v)>();
            
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[^(surface.DegreeU + 1)];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[^(surface.DegreeV + 1)];
            
            int gridSize = 30;
            double bestDist = double.PositiveInfinity;
            (double u, double v)? bestCandidate = null;
            
            // Sample interior points (skip exact boundaries)
            for (int i = 1; i < gridSize; i++)
            {
                double u = uMin + i * (uMax - uMin) / gridSize;
                for (int j = 1; j < gridSize; j++)
                {
                    double v = vMin + j * (vMax - vMin) / gridSize;
                    var point = surface.GetPos(u, v);
                    double dist = Math.Abs(plane.SignedDistanceTo(point));
                    
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestCandidate = (u, v);
                    }
                    
                    if (dist < tolerance * 50)
                    {
                        var refined = RefineIntersectionPoint(surface, plane, u, v, tolerance, 20);
                        if (refined.HasValue)
                        {
                            var tangent = GetIntersectionCurveTangent(surface, plane, refined.Value.u, refined.Value.v);
                            if (tangent.magnitude > 1e-8)
                            {
                                seeds.Add(refined.Value);
                                return seeds; // Found one good seed
                            }
                        }
                    }
                }
            }
            
            // If no close point found, try refining the best candidate
            if (bestCandidate.HasValue && bestDist < double.PositiveInfinity)
            {
                var refined = RefineIntersectionPoint(surface, plane, bestCandidate.Value.u, bestCandidate.Value.v, tolerance, 50);
                if (refined.HasValue)
                {
                    var tangent = GetIntersectionCurveTangent(surface, plane, refined.Value.u, refined.Value.v);
                    if (tangent.magnitude > 1e-8)
                    {
                        seeds.Add(refined.Value);
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