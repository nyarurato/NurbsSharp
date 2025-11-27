using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Intersector for NURBS curve and surface using Newton-Raphson iteration
    /// (ja) Newton-Raphson反復法を使用したNURBS曲線とサーフェスの交差判定
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
        /// (en) Find all intersections between a NURBS curve and surface using subdivision and Newton-Raphson
        /// (ja) 細分化とNewton-Raphson法を使用してNURBS曲線とサーフェス間の全交点を検索
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

            // Get parameter ranges
            double minU = curve.KnotVector.Knots[curve.Degree];
            double maxU = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];
            double minSU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxSU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minSV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxSV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            // Sample parameter space to find initial guesses
            int curveSamples = 30;
            int surfaceSamples = 20; // Increased samples for better detection
            var candidates = new List<(double u, double su, double sv, double dist)>();

            for (int i = 0; i <= curveSamples; i++)
            {
                double u = minU + (maxU - minU) * i / curveSamples;
                Vector3Double cp = CurveEvaluator.Evaluate(curve, u);

                for (int j = 0; j <= surfaceSamples; j++)
                {
                    double su = minSU + (maxSU - minSU) * j / surfaceSamples;
                    for (int k = 0; k <= surfaceSamples; k++)
                    {
                        double sv = minSV + (maxSV - minSV) * k / surfaceSamples;
                        Vector3Double sp = SurfaceEvaluator.Evaluate(surface, su, sv);

                        double dist = (cp - sp).magnitude;
                        if (dist < tolerance * 500) // Use larger tolerance for candidates
                        {
                            candidates.Add((u, su, sv, dist));
                        }
                    }
                }
            }

            // Refine each candidate using Newton-Raphson
            var refinedIntersections = new List<CurveSurfaceIntersection>();
            foreach (var candidate in candidates)
            {
                if (TryRefineIntersection(curve, surface, candidate.u, candidate.su, candidate.sv,
                    tolerance, out CurveSurfaceIntersection intersection))
                {
                    // Check if this intersection is unique (not duplicate)
                    bool isDuplicate = false;
                    foreach (var existing in refinedIntersections)
                    {
                        double paramDist = Math.Sqrt(
                            Math.Pow(existing.U - intersection.U, 2) +
                            Math.Pow(existing.SurfaceU - intersection.SurfaceU, 2) +
                            Math.Pow(existing.SurfaceV - intersection.SurfaceV, 2));
                        if (paramDist < tolerance * 10)
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
        /// (en) Refine an intersection point using Newton-Raphson iteration
        /// (ja) Newton-Raphson反復法を使用して交点を精密化
        /// </summary>
        /// <param name="curve">Curve to intersect</param>
        /// <param name="surface">Surface to intersect</param>
        /// <param name="initialU"></param>
        /// <param name="initialSU"></param>
        /// <param name="initialSV"></param>
        /// <param name="tolerance"></param>
        /// <param name="intersection"></param>
        private static bool TryRefineIntersection(NurbsCurve curve, NurbsSurface surface, double initialU, double initialSU, double initialSV, double tolerance, out CurveSurfaceIntersection intersection)
        {
            intersection = default;

            double u = initialU;
            double su = initialSU;
            double sv = initialSV;

            double minU = curve.KnotVector.Knots[curve.Degree];
            double maxU = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];
            double minSU = surface.KnotVectorU.Knots[surface.DegreeU];
            double maxSU = surface.KnotVectorU.Knots[surface.KnotVectorU.Length - surface.DegreeU - 1];
            double minSV = surface.KnotVectorV.Knots[surface.DegreeV];
            double maxSV = surface.KnotVectorV.Knots[surface.KnotVectorV.Length - surface.DegreeV - 1];

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                // Clamp parameters to valid range
                u = Math.Max(minU, Math.Min(maxU, u));
                su = Math.Max(minSU, Math.Min(maxSU, su));
                sv = Math.Max(minSV, Math.Min(maxSV, sv));

                // Evaluate positions
                Vector3Double cp = CurveEvaluator.Evaluate(curve, u);
                Vector3Double sp = SurfaceEvaluator.Evaluate(surface, su, sv);

                // Evaluate derivatives
                Vector3Double curveD = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                var surfaceDerivs = SurfaceEvaluator.EvaluateFirstDerivative(surface, su, sv);
                Vector3Double surfDU = surfaceDerivs.u_deriv; // ∂S/∂u
                Vector3Double surfDV = surfaceDerivs.v_deriv; // ∂S/∂v

                // Distance vector: f = C(u) - S(su, sv)
                Vector3Double f = cp - sp;
                double dist = f.magnitude;

                // Check convergence
                if (dist < tolerance)
                {
                    intersection = new CurveSurfaceIntersection
                    {
                        U = u,
                        SurfaceU = su,
                        SurfaceV = sv,
                        CurvePoint = cp,
                        SurfacePoint = sp,
                        Distance = dist
                    };
                    return true;
                }

                // Build Jacobian matrix for Newton-Raphson
                // We minimize ||C(u) - S(su,sv)||^2
                // F = [f·curveD, -f·surfDU, -f·surfDV]^T
                double f_dot_cd = Vector3Double.Dot(f, curveD);
                double f_dot_sdu = Vector3Double.Dot(f, surfDU);
                double f_dot_sdv = Vector3Double.Dot(f, surfDV);

                // Jacobian: 3x3 matrix
                // J = [[cd·cd,      -cd·sdu,    -cd·sdv   ],
                //      [-sdu·cd,    sdu·sdu,    sdu·sdv   ],
                //      [-sdv·cd,    sdv·sdu,    sdv·sdv   ]]
                double j11 = Vector3Double.Dot(curveD, curveD);
                double j12 = -Vector3Double.Dot(curveD, surfDU);
                double j13 = -Vector3Double.Dot(curveD, surfDV);
                double j21 = -Vector3Double.Dot(surfDU, curveD);
                double j22 = Vector3Double.Dot(surfDU, surfDU);
                double j23 = Vector3Double.Dot(surfDU, surfDV);
                double j31 = -Vector3Double.Dot(surfDV, curveD);
                double j32 = Vector3Double.Dot(surfDV, surfDU);
                double j33 = Vector3Double.Dot(surfDV, surfDV);

                // Solve J * delta = -F using Cramer's rule
                // F = [f_dot_cd, -f_dot_sdu, -f_dot_sdv]^T
                double det = j11 * (j22 * j33 - j23 * j32) -
                             j12 * (j21 * j33 - j23 * j31) +
                             j13 * (j21 * j32 - j22 * j31);

                if (Math.Abs(det) < 1e-12)
                {
                    // Singular matrix, cannot continue
                    return false;
                }

                // Cramer's rule for delta_u
                double det1 = -f_dot_cd * (j22 * j33 - j23 * j32) -
                              j12 * (f_dot_sdu * j33 - j23 * f_dot_sdv) +
                              j13 * (f_dot_sdu * j32 - j22 * f_dot_sdv);

                // Cramer's rule for delta_su
                double det2 = j11 * (f_dot_sdu * j33 - j23 * f_dot_sdv) -
                              (-f_dot_cd) * (j21 * j33 - j23 * j31) +
                              j13 * (j21 * f_dot_sdv - f_dot_sdu * j31);

                // Cramer's rule for delta_sv
                double det3 = j11 * (j22 * f_dot_sdv - f_dot_sdu * j32) -
                              j12 * (j21 * f_dot_sdv - f_dot_sdu * j31) +
                              (-f_dot_cd) * (j21 * j32 - j22 * j31);

                double deltaU = det1 / det;
                double deltaSU = det2 / det;
                double deltaSV = det3 / det;

                // Update parameters with damping for stability
                double dampingFactor = 1.0;
                if (iter < 5) dampingFactor = 0.5; // Conservative start

                u += dampingFactor * deltaU;
                su += dampingFactor * deltaSU;
                sv += dampingFactor * deltaSV;

                // Check if parameters went out of bounds
                if (u < minU - tolerance || u > maxU + tolerance ||
                    su < minSU - tolerance || su > maxSU + tolerance ||
                    sv < minSV - tolerance || sv > maxSV + tolerance)
                {
                    return false;
                }
            }

            // Did not converge
            return false;
        }

        /// <summary>
        /// (en) Check if a curve intersects a surface (boolean only)
        /// (ja) 曲線がサーフェスと交差するか判定（真偽値のみ）
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

        //TODO: marching method implementation
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
