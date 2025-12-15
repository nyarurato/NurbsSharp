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
    }
}