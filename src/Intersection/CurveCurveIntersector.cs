using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Intersector for two NURBS curves using Newton-Raphson iteration
    /// (ja) Newton-Raphson反復法を使用したNURBS曲線間の交差判定
    /// </summary>
    public static class CurveCurveIntersector
    {
        //TODO: Check for initial values and constatnts, iteration limits

        /// <summary>
        /// (en) Tolerance for intersection convergence
        /// (ja) 交点収束の許容誤差
        /// </summary>
        public const double Tolerance = 1e-6;

        /// <summary>
        /// (en) Maximum number of iterations for Newton-Raphson
        /// (ja) Newton-Raphson法の最大反復回数
        /// </summary>
        public const int MaxIterations = 1000;

        /// <summary>
        /// (en) Find all intersections between two NURBS curves using subdivision and Newton-Raphson
        /// (ja) 細分化とNewton-Raphson法を使用して2つのNURBS曲線間の全交点を検索
        /// </summary>
        /// <param name="curveA">First curve</param>
        /// <param name="curveB">Second curve</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>List of intersection points</returns>
        public static List<CurveCurveIntersection> Intersect(NurbsCurve curveA, NurbsCurve curveB, double tolerance = Tolerance)
        {
            Guard.ThrowIfNull(curveA, nameof(curveA));
            Guard.ThrowIfNull(curveB, nameof(curveB));

            var intersections = new List<CurveCurveIntersection>();

            // Early out: check bounding boxes
            if (!curveA.BoundingBox.Intersects(curveB.BoundingBox))
                return intersections;

            // Get parameter ranges
            double minU1 = curveA.KnotVector.Knots[curveA.Degree];
            double maxU1 = curveA.KnotVector.Knots[^(curveA.Degree+1)];// Last Degree knots
            double minU2 = curveB.KnotVector.Knots[curveB.Degree];
            double maxU2 = curveB.KnotVector.Knots[^(curveB.Degree+1)];// Last Degree knots


            // Sample parameter space to find initial guesses
            int samples = 100; // Number of samples per curve (increased for better detection)
            var candidates = new List<(double u1, double u2, double dist)>();

            for (int i = 0; i <= samples; i++)
            {
                double u1 = minU1 + (maxU1 - minU1) * i / samples;
                Vector3Double p1 = CurveEvaluator.Evaluate(curveA, u1);

                for (int j = 0; j <= samples; j++)
                {
                    double u2 = minU2 + (maxU2 - minU2) * j / samples;
                    Vector3Double p2 = CurveEvaluator.Evaluate(curveB, u2);

                    double dist = (p1 - p2).magnitude;
                    if (dist < 0.1) // Use larger tolerance for candidates (0.1 units)
                    {
                        candidates.Add((u1, u2, dist));
                    }
                }
            }

            // Refine each candidate using Newton-Raphson
            var refinedIntersections = new List<CurveCurveIntersection>();
            foreach (var (u1, u2, dist) in candidates)
            {
                if (TryRefineIntersection(curveA, curveB, u1, u2, tolerance, out CurveCurveIntersection intersection))
                {
                    // Check if this intersection is part of an existing cluster
                    bool merged = false;
                    for (int i = 0; i < refinedIntersections.Count; i++)
                    {
                        var existing = refinedIntersections[i];
                        double paramDist = Math.Sqrt(
                            Math.Pow(existing.U1 - intersection.U1, 2) +
                            Math.Pow(existing.U2 - intersection.U2, 2));

                        // Use a larger tolerance for clustering tangential intersections
                        // For tangential intersections, the "valid" region can be large (approx sqrt(tolerance))
                        if (paramDist < 1e-3)
                        {
                            merged = true;
                            // Keep the one with smaller distance (closer to true intersection)
                            if (intersection.Distance < existing.Distance)
                            {
                                refinedIntersections[i] = intersection;
                            }
                            break;
                        }
                    }

                    if (!merged)
                    {
                        refinedIntersections.Add(intersection);
                    }
                }
            }

            return refinedIntersections;
        }

        /// <summary>
        /// (en) Refine an intersection point using Newton-Raphson iteration
        /// (ja) Newton-Raphson反復法を使用して交点を精密化
        /// </summary>
        /// <param name="curveA"></param>
        /// <param name="curveB"></param>
        /// <param name="initialU1"></param>
        /// <param name="initialU2"></param>
        /// <param name="tolerance"></param>
        /// <param name="intersection"></param>
        private static bool TryRefineIntersection(NurbsCurve curveA, NurbsCurve curveB, double initialU1, double initialU2, double tolerance, out CurveCurveIntersection intersection)
        {
            intersection = default;

            double u1 = initialU1;
            double u2 = initialU2;

            double minU1 = curveA.KnotVector.Knots[curveA.Degree];
            double maxU1 = curveA.KnotVector.Knots[curveA.KnotVector.Length - curveA.Degree - 1];
            double minU2 = curveB.KnotVector.Knots[curveB.Degree];
            double maxU2 = curveB.KnotVector.Knots[curveB.KnotVector.Length - curveB.Degree - 1];

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                // Clamp parameters to valid range
                u1 = Math.Max(minU1, Math.Min(maxU1, u1));
                u2 = Math.Max(minU2, Math.Min(maxU2, u2));

                // Evaluate positions and derivatives
                Vector3Double p1 = CurveEvaluator.Evaluate(curveA, u1);
                Vector3Double p2 = CurveEvaluator.Evaluate(curveB, u2);
                Vector3Double d1 = CurveEvaluator.EvaluateFirstDerivative(curveA, u1);
                Vector3Double d2 = CurveEvaluator.EvaluateFirstDerivative(curveB, u2);

                // Distance vector
                Vector3Double f = p1 - p2;
                double dist = f.magnitude;

                // Check convergence
                if (dist < tolerance)
                {
                    intersection = new CurveCurveIntersection
                    {
                        U1 = u1,
                        U2 = u2,
                        Point1 = p1,
                        Point2 = p2,
                        Distance = dist
                    };
                    return true;
                }

                // Build Jacobian matrix for Newton-Raphson
                // We minimize ||C1(u1) - C2(u2)||^2
                // F = [f·d1, -f·d2]^T where f = C1(u1) - C2(u2)
                double f_dot_d1 = Vector3Double.Dot(f, d1);
                double f_dot_d2 = Vector3Double.Dot(f, d2);

                // Jacobian: J = [[d1·d1, -d1·d2], [-d2·d1, d2·d2]]
                double j11 = Vector3Double.Dot(d1, d1);
                double j12 = -Vector3Double.Dot(d1, d2);
                double j21 = -Vector3Double.Dot(d2, d1);
                double j22 = Vector3Double.Dot(d2, d2);

                // Determinant
                double det = j11 * j22 - j12 * j21;
                if (Math.Abs(det) < 1e-12)
                {
                    // Singular matrix, cannot continue
                    return false;
                }

                // Solve: J * delta = -F
                double deltaU1 = (-f_dot_d1 * j22 - f_dot_d2 * j12) / det;
                double deltaU2 = (f_dot_d2 * j11 + f_dot_d1 * j21) / det;

                // Update parameters with damping for stability
                double dampingFactor = 1.0;
                if (iter < 5) dampingFactor = 0.5; // Conservative start

                u1 += dampingFactor * deltaU1;
                u2 += dampingFactor * deltaU2;

                // Check if parameters went out of bounds
                if (u1 < minU1 - tolerance || u1 > maxU1 + tolerance ||
                    u2 < minU2 - tolerance || u2 > maxU2 + tolerance)
                {
                    return false;
                }
            }

            // Did not converge
            return false;
        }

        /// <summary>
        /// (en) Check if two curves intersect (boolean only)
        /// (ja) 2つの曲線が交差するか判定（真偽値のみ）
        /// </summary>
        /// <param name="curveA">First curve</param>
        /// <param name="curveB">Second curve</param>
        /// <param name="tolerance">Convergence tolerance</param>
        /// <returns>True if curves intersect</returns>
        public static bool Intersects(NurbsCurve curveA, NurbsCurve curveB, double tolerance = Tolerance)
        {
            var intersections = Intersect(curveA, curveB, tolerance);
            return intersections.Count > 0;
        }
    }

    /// <summary>
    /// (en) Result of curve-curve intersection
    /// (ja) 曲線-曲線交点の結果
    /// </summary>
    public struct CurveCurveIntersection
    {
        /// <summary>
        /// (en) Parameter on curve A
        /// (ja) 曲線Aのパラメータ
        /// </summary>
        public double U1 { get; set; }

        /// <summary>
        /// (en) Parameter on curve B
        /// (ja) 曲線Bのパラメータ
        /// </summary>
        public double U2 { get; set; }

        /// <summary>
        /// (en) Intersection point on curve A
        /// (ja) 曲線A上の交点
        /// </summary>
        public Vector3Double Point1 { get; set; }

        /// <summary>
        /// (en) Intersection point on curve B
        /// (ja) 曲線B上の交点
        /// </summary>
        public Vector3Double Point2 { get; set; }

        /// <summary>
        /// (en) Distance between the two points (should be near zero for true intersection)
        /// (ja) 2点間の距離（真の交点では0に近い）
        /// </summary>
        public double Distance { get; set; }
    }
}
