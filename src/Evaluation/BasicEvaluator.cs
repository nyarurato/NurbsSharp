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
    }
}
