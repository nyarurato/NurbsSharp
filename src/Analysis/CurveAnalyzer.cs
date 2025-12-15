using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Analysis
{
    /// <summary>
    /// (en) Analysis utilities for NURBS curves
    /// (ja) NURBS曲線の解析ユーティリティクラス
    /// </summary>
    public class CurveAnalyzer: BasicAnalyzer
    {
        /// <summary>
        /// (en) Calc the length of the NURBS curve
        /// (ja) NURBS曲線の長さを計算する
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="start_u"></param>
        /// <param name="end_u"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static double CurveLength(NurbsCurve curve, double start_u, double end_u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            if (start_u> end_u)
                throw new ArgumentOutOfRangeException(nameof(start_u), "start_u must be less than or equal to end_u.");
            if (start_u < curve.KnotVector.Knots[0])
                throw new ArgumentOutOfRangeException(nameof(start_u), "start_u is out of the knot vector range.");
            if (end_u > curve.KnotVector.Knots[curve.KnotVector.Length - 1])
                throw new ArgumentOutOfRangeException(nameof(end_u), "end_u is out of the knot vector range.");

            // Calculate the length of the NURBS curve using 5-point Gaussian quadrature

            int degree = curve.Degree;
            var knots = curve.KnotVector.Knots;
            if (knots == null || knots.Length < 2)
                return 0.0;

            // clamp integration range to valid evaluation domain
            double uMin = knots[degree];
            double uMax = knots[knots.Length - degree - 1];

            start_u = Math.Max(start_u, uMin);
            end_u = Math.Min(end_u, uMax);

            double total = 0.0;

            // integrate over each knot span to better capture local behavior
            for (int i = 0; i < knots.Length - 1; i++)
            {
                double a = Math.Max(start_u, knots[i]);
                double b = Math.Min(end_u, knots[i + 1]);

                if (b <= a)
                    continue;

                double half = 0.5 * (b - a);
                double center = 0.5 * (a + b);

                double spanSum = 0.0;
                for (int k = 0; k < GaussNode5.Length; k++)
                {
                    double xi = GaussNode5[k];
                    double wi = GaussWeight5[k];

                    double u = center + half * xi;// map from [-1,1] to [a,b] -> u = 1/2*((b-a)*xi + (a+b))

                    // Evaluate first derivative and take its magnitude
                    Vector3Double d = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                    double speed = d.magnitude;
                    spanSum += wi * speed;
                }

                total += half * spanSum; // multiply by Jacobian 0.5*(b-a)
            }

            return total;
        }


         /// <summary>
        /// (en) tangent, normal on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の接線、法線を評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        public static (Vector3Double tangent, Vector3Double normal) EvaluatTangentNormal(NurbsCurve curve, double u)
        {
            Vector3Double T =  CurveEvaluator.EvaluateFirstDerivative(curve, u);
            if (T.magnitude == 0.0)
                return (Vector3Double.Zero, Vector3Double.Zero);
            T = T.normalized;
            Vector3Double C2 = CurveEvaluator.EvaluateSecondDerivative(curve, u);
            if (C2.magnitude == 0.0)
                return (T, Vector3Double.Zero);
            double proj = Vector3Double.Dot(C2, T);
            Vector3Double normalVec = C2 - T * proj; // eliminate tangential component of C''
            if (normalVec.magnitude == 0.0)
                return (T, Vector3Double.Zero);
            Vector3Double N = normalVec.normalized;
            return (T, N);
        }

        /// <summary>
        /// (en) Evaluates the normal vector on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の法線ベクトルを評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        public static Vector3Double EvaluateNormal(NurbsCurve curve, double u)
        {
           return EvaluatTangentNormal(curve, u).normal;
        }

        /// <summary>
        /// (en) Evaluates the tangent vector on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の接線ベクトルを評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        public static Vector3Double EvaluateTangent(NurbsCurve curve, double u)
        {
            return EvaluatTangentNormal(curve, u).tangent;
        }

        /// <summary>
        /// (en) Evaluates the curvature on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の曲率を評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        public static double EvaluateCurvature(NurbsCurve curve, double u)
        {
            // κ = |C'(u) × C''(u)| / |C'(u)|^3
            Vector3Double firstDeriv = CurveEvaluator.EvaluateFirstDerivative(curve, u);
            var firstDerivMag = firstDeriv.magnitude;
            if(firstDerivMag == 0)
                return 0;            
            Vector3Double secondDeriv = CurveEvaluator.EvaluateSecondDerivative(curve, u);
            Vector3Double cross = Vector3Double.Cross(firstDeriv, secondDeriv);
            return cross.magnitude / Math.Pow(firstDerivMag,3);
        }

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