using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for NURBS curves
    /// (ja) NURBS曲線の操作クラス
    /// </summary>
    public static class CurveOperator
    {
        /// <summary>
        /// (en) Find the closest point on curve to a given 3D point using Newton-Raphson with grid search for initial point
        /// (ja) グリッド探索とNewton-Raphson法で指定された3D点に最も近い曲線上の点を検索
        /// </summary>
        /// <param name="curve">Target curve</param>
        /// <param name="target">Target 3D point</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <param name="gridDivisions">Number of grid divisions for initial point search (default: 20)</param>
        /// <returns>Tuple of (t parameter, point on curve, distance to target)</returns>
        public static (double t, Vector3Double point, double distance) FindClosestPoint(
            NurbsCurve curve,
            Vector3Double target,
            double tolerance = 1e-6,
            int gridDivisions = 20)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];

            // Grid search to find best starting point
            double bestT = minT;
            Vector3Double bestPoint = CurveEvaluator.Evaluate(curve, minT);
            double bestDistance = (bestPoint - target).magnitude;

            for (int i = 0; i <= gridDivisions; i++)
            {
                double t = minT + (maxT - minT) * i / gridDivisions;
                Vector3Double p = CurveEvaluator.Evaluate(curve, t);
                double dist = (p - target).magnitude;

                if (dist < bestDistance)
                {
                    bestDistance = dist;
                    bestT = t;
                    bestPoint = p;
                }
            }

            // Refine from best initial point using Newton-Raphson
            return FindClosestPointFromInitial(curve, target, bestT, tolerance, minT, maxT);
        }

        /// <summary>
        /// (en) Find the closest point on curve from a single initial guess using Newton-Raphson (fast)
        /// (ja) 単一の初期推定値からNewton-Raphson法で最近接点を検索（高速）
        /// </summary>
        /// <param name="curve">Target curve</param>
        /// <param name="target">Target 3D point</param>
        /// <param name="initialT">Initial guess for t parameter</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>Tuple of (t parameter, point on curve, distance to target)</returns>
        public static (double t, Vector3Double point, double distance) FindClosestPoint(
            NurbsCurve curve,
            Vector3Double target,
            double initialT,
            double tolerance = 1e-6)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            double minT = curve.KnotVector.Knots[curve.Degree];
            double maxT = curve.KnotVector.Knots[curve.KnotVector.Length - curve.Degree - 1];

            return FindClosestPointFromInitial(curve, target, initialT, tolerance, minT, maxT);
        }

        /// <summary>
        /// (en) Find closest point from a single initial guess using Newton-Raphson
        /// (ja) 単一の初期点からNewton-Raphson法で最近接点を検索
        /// </summary>
        private static (double t, Vector3Double point, double distance) FindClosestPointFromInitial(
            NurbsCurve curve,
            Vector3Double target,
            double initialT,
            double tolerance,
            double minT,
            double maxT)
        {
            double t = Math.Max(minT, Math.Min(maxT, initialT));
            const int maxIterations = 100;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                Vector3Double C = CurveEvaluator.Evaluate(curve, t);
                Vector3Double Cp = CurveEvaluator.EvaluateFirstDerivative(curve, t);
                Vector3Double Cpp = curve.Degree >= 2 
                    ? CurveEvaluator.EvaluateSecondDerivative(curve, t) 
                    : Vector3Double.Zero;

                Vector3Double diff = C - target;
                double dist = diff.magnitude;

                // Check convergence
                if (dist < tolerance)
                {
                    return (t, C, dist);
                }

                // Minimize ||C(t) - target||^2
                // Gradient: g = 2 * (C - target) · C'
                double g = 2.0 * Vector3Double.Dot(diff, Cp);

                // Second derivative: g' = 2 * (C' · C' + (C - target) · C'')
                double gPrime = 2.0 * (Vector3Double.Dot(Cp, Cp) + Vector3Double.Dot(diff, Cpp));

                if (Math.Abs(gPrime) < 1e-12)
                    break;

                // Newton step
                double dt = -g / gPrime;

                // Apply damping for stability
                double damping = 0.7;
                t += damping * dt;

                // Clamp to bounds
                t = Math.Max(minT, Math.Min(maxT, t));

                // Check parameter convergence
                if (Math.Abs(dt) < tolerance)
                {
                    Vector3Double finalC = CurveEvaluator.Evaluate(curve, t);
                    double finalDist = (finalC - target).magnitude;
                    return (t, finalC, finalDist);
                }
            }

            // Return best result after max iterations
            Vector3Double finalPoint = CurveEvaluator.Evaluate(curve, t);
            double finalDistance = (finalPoint - target).magnitude;
            return (t, finalPoint, finalDistance);
        }
    }
}
