using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System.Numerics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace NurbsSharp.Evaluation
{
    /// <summary>
    /// (en) Evaluator for NURBS curves
    /// (ja) NURBS曲線の評価クラス
    /// </summary>
    public class CurveEvaluator:BasicEvaluator
    {
        /// <summary>
        /// (en) Evaluates the position on the NURBS curve at the specified parameter u. The range is the same as the knot vector's minimum and maximum values.
        /// (ja) 指定したパラメータ u でNURBS曲線上の位置を評価します。レンジはノットベクトルの最小値と最大値と同じです。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        /// <exception cref="ArgumentException"></exception>
        public static Vector3Double Evaluate(NurbsCurve curve, double u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            if (curve.KnotVector.Knots[0] > u || curve.KnotVector.Knots[curve.KnotVector.Length - 1] < u)
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' must be in the range [0, 1].");
            if (curve.Degree < 1)
                throw new ArgumentException("Curve degree must be at least 1.", nameof(curve));

            // ****** de Boor's algorithm *******
            int degree = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;

            if(u < knots[degree])
                u = knots[degree];
            else if (u > knots[knots.Length - degree - 1])
                u = knots[knots.Length - degree - 1];

            int k = FindSpan(degree, knots, u);

            Vector4Double[][] d = new Vector4Double[degree + 1][];
            //initialize d
            for (int i = 0; i <= degree; i++)
            {
                d[i] = new Vector4Double[degree + 1];
            }
            
            // convert back to 3D
            Vector4Double resultH = DeBoor(degree, knots, k, controlPoints.Select(cp => cp.HomogeneousPosition).ToArray(), u);
            return new Vector3Double(resultH.X / resultH.W, resultH.Y / resultH.W, resultH.Z / resultH.W);
        }

        /// <summary>
        /// (en) Evaluates the homogeneous position (weighted) on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の同次座標（重み付き）位置を評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        public static Vector4Double EvaluateHomogeneous(NurbsCurve curve, double u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            if (curve.KnotVector.Knots[0] > u || curve.KnotVector.Knots[curve.KnotVector.Length - 1] < u)
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' must be in the range [0, 1].");
            if (curve.Degree < 1)
                throw new ArgumentException("Curve degree must be at least 1.", nameof(curve));

            int degree = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;

            if (u < knots[degree])
                u = knots[degree];
            else if (u > knots[knots.Length - degree - 1])
                u = knots[knots.Length - degree - 1];

            int k = FindSpan(degree, knots, u);

            return DeBoor(degree, knots, k, controlPoints.Select(cp => cp.HomogeneousPosition).ToArray(), u);
        }

        /// <summary>
        /// (en) Evaluates the first derivative vector on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の一階微分ベクトルを評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        public static Vector3Double EvaluateFirstDerivative(NurbsCurve curve,double u)
        {
            //TODO: Optimize Deboor-based derivative evaluation
            Guard.ThrowIfNull(curve, nameof(curve));

            int degree = curve.Degree;
            if (degree < 1)
                throw new ArgumentException("Curve degree must be at least 1 for derivative", nameof(curve));

            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;
            if (controlPoints.Length < 2)
                throw new ArgumentException("At least two control points are required", nameof(curve));

            int span = FindSpan(degree, knots, u);
            int first = Math.Max(0, span - degree);
            int last = Math.Min(span, controlPoints.Length - 1);

            int n = controlPoints.Length - 1;
            double denom = 0.0;
            Vector3Double weightPosDerivSum = new (0.0, 0.0, 0.0); //ΣwPN' 
            Vector3Double weightPosSum = new (0.0, 0.0, 0.0);//ΣwPN
            double sumWeightBasis=0, sumWeightBasisDeriv=0;//ΣwN and ΣwN'
            double N = 0.0, dN=0;

            // Compute the derivative control points
            for (int i= first; i <= last; i++)
            {
                N = BSplineBasisFunction(i, degree, u,knots);
                dN = DerivativeBSplineBasisFunction(i, degree, u, knots);
                denom += controlPoints[i].Weight * N;

                weightPosDerivSum += controlPoints[i].Weight * controlPoints[i].Position * dN;//wPN'
                sumWeightBasis += controlPoints[i].Weight * N;//wN

                weightPosSum += controlPoints[i].Weight * controlPoints[i].Position * N;//wPN
                sumWeightBasisDeriv += controlPoints[i].Weight * dN;//wN'
            }
            
            if(denom == 0)
                return new Vector3Double(0.0,0.0,0.0);

            Vector3Double result = (weightPosDerivSum*sumWeightBasis - weightPosSum*sumWeightBasisDeriv)/(denom * denom);

            return result;

        }

        /// <summary>
        /// (en) Evaluates the second derivative vector on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の二階微分ベクトルを評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        public static Vector3Double EvaluateSecondDerivative(NurbsCurve curve, double u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            int degree = curve.Degree;
            if (degree < 2)
                throw new ArgumentException("Curve degree must be at least 2 for second derivative", nameof(curve));

            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;
            if (controlPoints.Length < 2)
                throw new ArgumentException("At least two control points are required", nameof(curve));
            
            int n = controlPoints.Length - 1;
            int span = FindSpan(degree, knots, u);
            int first = Math.Max(0, span - degree);
            int last = Math.Min(span, controlPoints.Length - 1);

            Vector3Double A = new(), A_deriv = new(), A_deriv2 = new();
            double W = 0.0, W_deriv= 0.0, W_deriv2 = 0.0;

            // Compute the derivative control points
            for (int i = first; i <= last; i++) { 
                var N = BSplineBasisFunction(i, degree, u, knots);
                var dN = DerivativeBSplineBasisFunction(i, degree, u, knots);
                var d2N = DerivativeBSplineBasisFunction(i, degree, u, knots, 2);

                A += controlPoints[i].Weight * controlPoints[i].Position * N; // wP N
                A_deriv += controlPoints[i].Weight * controlPoints[i].Position * dN; // wP N'
                A_deriv2 += controlPoints[i].Weight * controlPoints[i].Position * d2N; // wP N''
                W += controlPoints[i].Weight * N; // w N
                W_deriv += controlPoints[i].Weight * dN; // w N'
                W_deriv2 += controlPoints[i].Weight * d2N; // w N''
            }
            //C2 = (A'' * W^2 - 2 A' W W' - A W W'' + 2 A (W')^2) / (W^3)
            Vector3Double secondDeriv = (A_deriv2 * W * W - 2 * A_deriv * W * W_deriv - A * W * W_deriv2 + 2 * A * W_deriv * W_deriv) / (W * W * W);

            return secondDeriv;
        }


        /// <summary>
        /// (en) Calc the length of the NURBS curve
        /// (ja) NURBS曲線の長さを計算する
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="start_u"></param>
        /// <param name="end_u"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        [Obsolete("Use CurveAnalyzer.CurveLength instead.")]
        public static double CurveLength(NurbsCurve curve, double start_u, double end_u)
        {
            return Analysis.CurveAnalyzer.CurveLength(curve, start_u, end_u);
        }

        /// <summary>
        /// (en) tangent, normal on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の接線、法線を評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        [Obsolete("Use CurveAnalyzer.EvaluatTangentNormal instead.")]
        public static (Vector3Double tangent, Vector3Double normal) EvaluatTangentNormal(NurbsCurve curve, double u)
        {
            return Analysis.CurveAnalyzer.EvaluateTangentNormal(curve, u);
        }

        /// <summary>
        /// (en) Evaluates the curvature on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の曲率を評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        [Obsolete("Use CurveAnalyzer.EvaluateCurvature instead.")]
        public static double EvaluateCurvature(NurbsCurve curve, double u)
        {
            return Analysis.CurveAnalyzer.EvaluateCurvature(curve, u);
        }

        /// <summary>
        /// (en) Evaluates the normal vector on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の法線ベクトルを評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        [Obsolete("Use CurveAnalyzer.EvaluateNormal instead.")]
        public static Vector3Double EvaluateNormal(NurbsCurve curve, double u)
        {
           return Analysis.CurveAnalyzer.EvaluateNormal(curve, u);
        }

        /// <summary>
        /// (en) Evaluates the tangent vector on the NURBS curve at the specified parameter u.
        /// (ja) 指定したパラメータ u でNURBS曲線上の接線ベクトルを評価します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        [Obsolete("Use CurveAnalyzer.EvaluateTangent instead.")]
        public static Vector3Double EvaluateTangent(NurbsCurve curve, double u)
        {
            return Analysis.CurveAnalyzer.EvaluateTangent(curve, u);
        }



    }
}
