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
        public static (double x, double y, double z) Evaluate(NurbsCurve curve, double u)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));
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
            return (resultH.X / resultH.W, resultH.Y / resultH.W, resultH.Z / resultH.W);
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
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));

            int degree = curve.Degree;
            if (degree < 1)
                throw new ArgumentException("Curve degree must be at least 1 for derivative", nameof(curve));

            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;
            if (controlPoints.Length < 2)
                throw new ArgumentException("At least two control points are required", nameof(curve));

            int n = controlPoints.Length - 1;
            double denom = 0.0;
            Vector3Double weightPosDerivSum, weightPosSum;//ΣwPN' and ΣwPN
            double sumWeightBasis=0, sumWeightBasisDeriv=0;//ΣwN and ΣwN'
            double N = 0.0, dN=0;

            weightPosDerivSum = new Vector3Double(0.0, 0.0, 0.0);
            weightPosSum = new Vector3Double(0.0, 0.0, 0.0);

            // Compute the derivative control points
            for (int i= 0; i <= n; i++)
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
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));
            int degree = curve.Degree;
            if (degree < 2)
                throw new ArgumentException("Curve degree must be at least 2 for second derivative", nameof(curve));

            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;
            if (controlPoints.Length < 2)
                throw new ArgumentException("At least two control points are required", nameof(curve));
            
            int n = controlPoints.Length - 1;

            Vector3Double A = new(), A_deriv = new(), A_deriv2 = new();
            double W = 0.0, W_deriv= 0.0, W_deriv2 = 0.0;

            // Compute the derivative control points
            for (int i = 0; i <= n; i++) { 
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


        // TODO: Optimize using different numerical integration methods
        /// <summary>
        /// (en) Calc the length of the NURBS curve
        /// (ja) NURBS曲線の長さを計算する
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="start_u"></param>
        /// <param name="end_u"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static double CurveLength(NurbsCurve curve, double start_u, double end_u, double epsilon = 0.001)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));

            double length = 0.0;
            Vector3Double prevPoint = new Vector3Double();
            bool isFirstPoint = true;

            for (double u = start_u; u <= end_u; u += epsilon)
            {
                var pos = Evaluate(curve, u);
                Vector3Double currentPoint = new Vector3Double(pos.x, pos.y, pos.z);
                if (isFirstPoint)
                {
                    prevPoint = currentPoint;
                    isFirstPoint = false;
                }
                else
                {
                    length += prevPoint.DistanceTo(currentPoint);
                    prevPoint = currentPoint;
                }
            }

            return length;
        }

    }
}
