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
    /// <summary>
    /// (en) Evaluator for NURBS surfaces
    /// (ja) NURBSサーフェスの評価クラス
    /// </summary>
    public class SurfaceEvaluator:BasicEvaluator
    {
        /// <summary>
        /// (en) Evaluates the position on the NURBS surface at the specified parameters u and v. The range is the same as the knot vector's minimum and maximum values.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の位置を評価します。レンジはノットベクトルの最小値と最大値と同じです。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
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

            // de Boor's algorithm in V direction
            Vector4Double Sv = DeBoor(degreeV, knotsV,spanV, temp, v);

            return (Sv.X/Sv.W, Sv.Y / Sv.W, Sv.Z / Sv.W);
        }

        //TODO: Optimize using different numerical integration methods
        // not good for high curvature surface
        /// <summary>
        /// (en) Calc Area of the NURBS Surface
        /// (ja) NURBSサーフェスの面積を計算します
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="start_u"></param>
        /// <param name="end_u"></param>
        /// <param name="start_v"></param>
        /// <param name="end_v"></param>
        /// <param name="epsilon"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static double SurfaceArea(NurbsSurface surface, double start_u, double end_u, double start_v, double end_v, double epsilon = 0.01)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));
            double area = 0.0;
            for (double u = start_u; u < end_u; u += epsilon)
            {

                for (double v = start_v; v < end_v; v += epsilon)
                {
                    Vector3Double p1, p2, p3,p4;
                    //pick small triangle area
                    var res = Evaluate(surface, u, v);
                    p1 = new Vector3Double(res.x, res.y, res.z);

                    if (u + epsilon > end_u)
                        res = Evaluate(surface, end_u, v);
                    else
                        res = Evaluate(surface, u + epsilon, v);
                    p2 = new Vector3Double(res.x, res.y, res.z);
                    
                    if (v + epsilon > end_v)
                        res = Evaluate(surface, u, end_v);
                    else
                        res = Evaluate(surface, u, v + epsilon);
                    p3 = new Vector3Double(res.x, res.y, res.z);

                    if (u + epsilon > end_u)
                    {
                        if (v + epsilon > end_v)
                            res = Evaluate(surface, end_u, end_v);
                        else
                            res = Evaluate(surface, end_u, v + epsilon);
                    }
                    else if(v + epsilon > end_v)
                        res = Evaluate(surface, u + epsilon, end_v);
                    else
                        res = Evaluate(surface, u + epsilon, v + epsilon);

                    p4 = new Vector3Double(res.x, res.y, res.z);
                    // S = ||AB x AC||
                    Vector3Double vec1 = p2 - p1;
                    Vector3Double vec2 = p3 - p1;
                    Vector3Double crossProduct = Vector3Double.Cross(vec1, vec2);
                    area += crossProduct.magnitude / 2; // Triangle area

                    vec1 = p2 - p4;
                    vec2 = p3 - p4;
                    crossProduct = Vector3Double.Cross(vec1, vec2);

                    area += crossProduct.magnitude / 2; // Triangle area

                }
            }
            return area;
        }

        /// <summary>
        /// (en) Evaluates the first derivatives of the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェスの1階微分を評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        /// <exception cref="DivideByZeroException"></exception>
        public static (Vector3Double u_deriv,Vector3Double v_deriv) EvaluateFirstDerivative(NurbsSurface surface, double u, double v)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));

            int p = surface.DegreeU;
            int q = surface.DegreeV;
            if (p < 0 || q < 0)
                throw new ArgumentException("Surface degrees must be non-negative.", nameof(surface));

            var cps = surface.ControlPoints;
            int nu = cps.Length;           // #CP in U
            int nv = cps[0].Length;        // #CP in V

            double[] U = surface.KnotVectorU.Knots;
            double[] V = surface.KnotVectorV.Knots;

            // 1) パラメータを有効ドメインにクランプ（最終ノットは半開区間対策で 1ULP 手前へ）
            double umin = U[p];
            double umax = U[U.Length - p - 1];
            double vmin = V[q];
            double vmax = V[V.Length - q - 1];

            if (u < umin) u = umin;
            if (u > umax) u = LinAlg.BitDecrement(umax);
            if (v < vmin) v = vmin;
            if (v > vmax) v = LinAlg.BitDecrement(vmax);

            // 2) スパンを取得
            int spanU = FindSpan(p, U, u);
            int spanV = FindSpan(q, V, v);

            int iu0 = spanU - p;
            int iv0 = spanV - q;

            // 3) 局所基底とその1階微分のみを計算
            double[] Nu = new double[p + 1];
            double[] Nu_d = new double[p + 1];
            for (int k = 0; k <= p; k++)
            {
                int i = iu0 + k;
                Nu[k]   = BSplineBasisFunction(i, p, u, U);
                Nu_d[k] = DerivativeBSplineBasisFunction(i, p, u, U);
            }

            double[] Nv = new double[q + 1];
            double[] Nv_d = new double[q + 1];
            for (int l = 0; l <= q; l++)
            {
                int j = iv0 + l;
                Nv[l]   = BSplineBasisFunction(j, q, v, V);
                Nv_d[l] = DerivativeBSplineBasisFunction(j, q, v, V);
            }

            // 4) 重み付き和
            Vector3Double A  = Vector3Double.Zero;
            Vector3Double Au = Vector3Double.Zero;
            Vector3Double Av = Vector3Double.Zero;
            double W = 0.0, Wu = 0.0, Wv = 0.0;

            for (int k = 0; k <= p; k++)
            {
                int i = iu0 + k;
                for (int l = 0; l <= q; l++)
                {
                    int j = iv0 + l;

                    ControlPoint cp = cps[i][j];
                    double w = cp.Weight;
                    Vector3Double Pw = cp.Position * w;

                    double Nuv    = Nu[k]   * Nv[l];
                    double Nu_dNv = Nu_d[k] * Nv[l];
                    double NuNv_d = Nu[k]   * Nv_d[l];

                    A  += Pw * Nuv;
                    W  += w  * Nuv;

                    Au += Pw * Nu_dNv;
                    Wu += w  * Nu_dNv;

                    Av += Pw * NuNv_d;
                    Wv += w  * NuNv_d;
                }
            }

            if (W == 0.0)
                throw new DivideByZeroException("The weight function evaluated to zero.");
                //return (Vector3Double.Zero,Vector3Double.Zero);

            var resultU = (Au * W - A * Wu) / (W * W);
            
            var resultV = (Av * W - A * Wv) / (W * W);

            return (resultU,resultV);
        }
    }

}
