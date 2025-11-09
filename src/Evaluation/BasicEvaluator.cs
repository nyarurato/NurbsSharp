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
            int k = -1;
            int s = 0; // multiplicity of u
            if (u < knots[degree])
            {
                k = degree;
            }
            else
            {
                for (int i = degree; i < (knots.Length - 1); i++)
                {
                    if (u >= knots[i] && u < knots[i + 1])
                    {
                        k = i;
                        break;
                    }
                    else if (u == knots[i + 1])
                    {
                        s++;
                    }
                }
                if (u == knots[knots.Length - 1])
                {
                    k = knots.Length - s - 1;
                }
            }

            if (k == -1)
            {
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' is out of the knot vector range.");
            }
            return k;

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
                    if (knots[span + 1 + j - r] != knots[span - p + j])
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
                if (denom1 != 0)
                    term1 = (u - knots[i]) / denom1 * BSplineBasisFunction(i, p - 1, u, knots);
                if (denom2 != 0)
                    term2 = (knots[i + p + 1] - u) / denom2 * BSplineBasisFunction(i + 1, p - 1, u, knots);
                return term1 + term2;
            }
        }
        /// <summary>
        /// (en) Derivative of Bspline basis function
        /// (ja) Bspline基底関数の微分
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
                if (denom1 != 0)
                    term1 = p / denom1 * BSplineBasisFunction(i, p - 1, u, knots);
                if (denom2 != 0)
                    term2 = -p / denom2 * BSplineBasisFunction(i + 1, p - 1, u, knots);
                return term1 + term2;
            }
        }



    }

}
