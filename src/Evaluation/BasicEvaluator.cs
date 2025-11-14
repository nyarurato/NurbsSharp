using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Evaluation
{
    /// <summary>
    /// (en) Basic evaluator for NURBS curves and surfaces
    /// (ja) NURBSの基本的な評価用基底クラス
    /// </summary>
    public class BasicEvaluator
    {
        /// <summary>
        /// (en) 5-point Gauss-Legendre quadrature nodes
        /// (ja) 5点ガウス・ルジャンドル求積法のノード
        /// </summary>
        protected static readonly double[] GaussNode5 = [
            -0.906179845938664,
            -0.538469310105683,
            0,
            0.538469310105683,
            0.906179845938664
        ];
        /// <summary>
        /// (en) 5-point Gauss-Legendre quadrature weights
        /// (ja) 5点ガウス・ルジャンドル求積法の重み
        /// </summary>
        protected static readonly double[] GaussWeight5 = [
            0.236926885056189,
            0.478628670499366,
            0.568888888888889,
            0.478628670499366,
            0.236926885056189
        ];

        /// <summary>
        /// (en) Find the knot span index for the given parameter u
        /// (ja) 指定したパラメータ u に対するノットベクトルの区間インデックスを見つける
        /// </summary>
        /// <param name="degree"></param>
        /// <param name="knots"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        protected static int FindSpan(int degree, double[] knots, double u)
        {
            // n: last control point index
            int n = knots.Length - degree - 2;
            if (n < 0)
                throw new ArgumentOutOfRangeException(nameof(knots), "Invalid knot vector or degree.");

            // Clamp to valid domain [U[p], U[n+1]] with robust end handling
            if (u <= knots[degree]) return degree;
            if (u >= knots[n + 1]) return n;

            int low = degree;
            int high = n + 1; // invariant: knots[low] <= u < knots[high]
            int mid = (low + high) / 2;

            while (u < knots[mid] || u >= knots[mid + 1])
            {
                if (u < knots[mid])
                    high = mid;
                else
                    low = mid;

                mid = (low + high) / 2;
            }
            return mid;

        }

        /// <summary>
        /// (en) de Boor's algorithm to evaluate the point on the NURBS
        /// (ja) de BoorのアルゴリズムによるNURBS上の点の評価
        /// </summary>
        /// <param name="p">degree</param>
        /// <param name="knots">knots vector</param>
        /// <param name="span_i">knots span index</param>
        /// <param name="ctrlPoints">control point of NURBS</param>
        /// <param name="u">parameter u</param>
        /// <returns></returns>
        protected static Vector4Double DeBoor(int p, double[] knots, int span_i, Vector4Double[] ctrlPoints, double u)
        {

            int n = ctrlPoints.Length - 1;
            // Find span index
            int span = span_i;

            // initalize
            Vector4Double[] d = new Vector4Double[p + 1];
            for (int j = 0; j <= p; j++)
            {
                d[j] = ctrlPoints[span - p + j];
            }

            //calculation
            for (int r = 1; r <= p; r++)
            {
                for (int j = p; j >= r; j--)
                {
                    double alpha = 0;
                    if (LinAlg.ApproxNotEqual(knots[span + 1 + j - r], knots[span - p + j]))
                        alpha = (u - knots[span - p + j]) / (knots[span + 1 + j - r] - knots[span - p + j]);
                    d[j] = (1 - alpha) * d[j - 1] + alpha * d[j];
                }
            }

            return d[p];
        }

        /// <summary>
        /// (en)Bspline basis function N_{i,p}(u)
        /// (ja)Bspline基底関数 N_{i,p}(u)
        /// </summary>
        /// <param name="i"></param>
        /// <param name="p">degree</param>
        /// <param name="u">parameter u</param>
        /// <param name="knots"></param>
        /// <returns></returns>
        protected static double BSplineBasisFunction(int i, int p, double u, double[] knots)
        {
    
            if (LinAlg.ApproxEqual(u, knots[knots.Length - 1]))
            {
                if(i == knots.Length - p - 2)
                    return 1.0;
                else
                    return 0.0;
            }

            /*
             * N_{i,0}(u) = { 1 if u_i <= u < u_{i+1}
             * N_{i,p}(u) = (u - u_i)/(u_{i+p} - u_i) * N_{i,p-1}(u) + (u_{i+p+1} - u)/(u_{i+p+1} - u_{i+1}) * N_{i+1,p-1}(u) 
             */
            if (p == 0)
            {
                if (knots[i] <= u && u < knots[i + 1])
                    return 1.0;
                else
                    return 0.0;
            }
            else
            {
                double denom1 = knots[i + p] - knots[i];
                double denom2 = knots[i + p + 1] - knots[i + 1];
                double term1 = 0.0;
                double term2 = 0.0;
                if (LinAlg.IsNotZero(denom1))
                    term1 = (u - knots[i]) / denom1 * BSplineBasisFunction(i, p - 1, u, knots);
                if (LinAlg.IsNotZero(denom2))
                    term2 = (knots[i + p + 1] - u) / denom2 * BSplineBasisFunction(i + 1, p - 1, u, knots);
                return term1 + term2;
            }
        }
        /// <summary>
        /// (en) First derivative of Bspline basis function N'_{i,p}(u)
        /// (ja) Bspline基底関数の1階微分 N'_{i,p}(u)
        /// </summary>
        /// <param name="i"></param>
        /// <param name="p"></param>
        /// <param name="u"></param>
        /// <param name="knots"></param>
        /// <returns></returns>
        protected static double DerivativeBSplineBasisFunction(int i, int p, double u, double[] knots)
        {
            /*
             * dN_{i,p}(u)/du = p/(u_{i+p} - u_i) * N_{i,p-1}(u) - p/(u_{i+p+1} - u_{i+1}) * N_{i+1,p-1}(u) 
             */
            if (p == 0)
            {
                return 0.0;
            }
            else
            {
                double denom1 = knots[i + p] - knots[i];
                double denom2 = knots[i + p + 1] - knots[i + 1];
                double term1 = 0.0;
                double term2 = 0.0;
                if (LinAlg.IsNotZero(denom1))
                    term1 = p / denom1 * BSplineBasisFunction(i, p - 1, u, knots);// p/(u_{i+p} - u_i) * N_{i,p-1}(u)
                if (LinAlg.IsNotZero(denom2))
                    term2 = -p / denom2 * BSplineBasisFunction(i + 1, p - 1, u, knots);//- p/(u_{i+p+1} - u_{i+1}) * N_{i+1,p-1}(u) 
                return term1 + term2;
            }
        }

        /// <summary>
        /// (en) Derivative of Bspline basis function of given order
        /// (ja) 指定した階数のBspline基底関数の微分
        /// </summary>
        /// <param name="i"></param>
        /// <param name="p"></param>
        /// <param name="u"></param>
        /// <param name="knots"></param>
        /// <param name="order"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        protected static double DerivativeBSplineBasisFunction(int i, int p, double u, double[] knots,int order=1)
        {
            if(order < 1)
                throw new ArgumentOutOfRangeException(nameof(order), "Order must be at least 1.");
            if(order ==1)
                return DerivativeBSplineBasisFunction(i, p, u, knots);
            else
            {
                // 2nd or higher order derivative
                // Iteratively apply the derivative formula

                double denom1 = knots[i + p] - knots[i];
                double denom2 = knots[i + p + 1] - knots[i + 1];
                double term1 = 0.0;
                double term2 = 0.0;
                if (LinAlg.IsNotZero(denom1))
                    term1 = p / denom1 * DerivativeBSplineBasisFunction(i, p - 1, u, knots, order - 1);
                if (LinAlg.IsNotZero(denom2))
                    term2 = -p / denom2 * DerivativeBSplineBasisFunction(i + 1, p - 1, u, knots, order - 1);
                return term1 + term2;

            }
        }
    }

}
