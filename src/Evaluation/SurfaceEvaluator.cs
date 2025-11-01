using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Evaluation
{
    public class SurfaceEvaluator
    {
        public static (double x, double y, double z) Evaluate(NurbsSurface surface, double u, double v)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;

            var controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;

            var knotsU = surface.KnotVectorU.Knots;
            var knotsV = surface.KnotVectorV.Knots;

            // find u v knot span index
            int spanU = FindSpan(degreeU, knotsU, u);
            int spanV = FindSpan(degreeV, knotsV, v);

            // de Boor's algorithm in U direction
            Vector4Double[] temp = new Vector4Double[nV];
            for (int j = 0; j < nV; j++)
            {
                Vector4Double[] row = new Vector4Double[nU];
                for (int i = 0; i < nU; i++)
                {
                    row[i] = controlPoints[i][j].HomogeneousPosition;
                }
                temp[j] = DeBoor(degreeU, knotsU,spanU, row, u);
            }

            // 2. v方向で補間
            Vector4Double Sv = DeBoor(degreeV, knotsV,spanV, temp, v);

            return (Sv.X/Sv.W, Sv.Y / Sv.W, Sv.Z / Sv.W);
        }

        // Find knot span index
        static int FindSpan(int degree, double[] knots, double u)
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

        public static Vector4Double DeBoor(int p, double[] knots,int span_i, Vector4Double[] ctrlPoints, double u)
        {

            int n = ctrlPoints.Length - 1;
            // ノットスパンを検索
            int span = span_i;

            // 初期化
            Vector4Double[] d = new Vector4Double[p + 1];
            for (int j = 0; j <= p; j++)
            {
                d[j] = ctrlPoints[span - p + j];
            }

            // 再帰計算
            for (int r = 1; r <= p; r++)
            {
                for (int j = p; j >= r; j--)
                {
                    double alpha = 0;
                    if(knots[span + 1 + j - r] != knots[span - p + j])
                        alpha =(u - knots[span - p + j]) / (knots[span + 1 + j - r] - knots[span - p + j]);
                    d[j] = (1 - alpha) * d[j - 1] + alpha * d[j];
                }
            }

            return d[p];
        }
    }

}
