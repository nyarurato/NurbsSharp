using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

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
        public static Vector3Double Evaluate(NurbsSurface surface, double u, double v)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            //prameter range check, (clamped knot vector range)
            if (u < surface.KnotVectorU.Knots[surface.DegreeU] || u > surface.KnotVectorU.Knots[^surface.DegreeU])
                throw new ArgumentOutOfRangeException(nameof(u),$"Parameter u is out of knot vector range. u:{u} must in [{surface.KnotVectorU.Knots[surface.DegreeU]},{surface.KnotVectorU.Knots[^surface.DegreeU]}]");
            if (v < surface.KnotVectorV.Knots[surface.DegreeV] || v > surface.KnotVectorV.Knots[^surface.DegreeV])
                throw new ArgumentOutOfRangeException(nameof(v), $"Parameter v is outof knot vector range. v:{v} must in [{surface.KnotVectorV.Knots[surface.DegreeV]},{surface.KnotVectorV.Knots[^surface.DegreeV]}]");

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

            return new Vector3Double(Sv.X/Sv.W, Sv.Y / Sv.W, Sv.Z / Sv.W);
        }

        // Current accuracy: ~0.0013 absolute error for sphere (radius=7.5)
        // Relative error: ~0.00018% - sufficient for most CAD/CAM applications
        // Further improvement would require higher-order quadrature or adaptive integration
        /// <summary>
        /// (en) Calc Area of the NURBS Surface
        /// (ja) NURBSサーフェスの面積を計算します
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="start_u"></param>
        /// <param name="end_u"></param>
        /// <param name="start_v"></param>
        /// <param name="end_v"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        [Obsolete("Use Analysis.SurfaceAnalyzer.SurfaceArea instead.")]
        public static double SurfaceArea(NurbsSurface surface, double start_u, double end_u, double start_v, double end_v)
        {
            return Analysis.SurfaceAnalyzer.SurfaceArea(surface, start_u, end_u, start_v, end_v);
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
            Guard.ThrowIfNull(surface, nameof(surface));

            int p = surface.DegreeU;
            int q = surface.DegreeV;
            if (p < 0 || q < 0)
                throw new ArgumentException("Surface degrees must be non-negative.", nameof(surface));

            var cps = surface.ControlPoints;
            int nu = cps.Length;           // #CP in U
            int nv = cps[0].Length;        // #CP in V

            double[] U = surface.KnotVectorU.Knots;
            double[] V = surface.KnotVectorV.Knots;

            // Clamp u and v to the valid range
            double umin = U[p];
            double umax = U[U.Length - p - 1];
            double vmin = V[q];
            double vmax = V[V.Length - q - 1];

            if (u < umin) u = umin;
            if (u > umax) u = LinAlg.BitDecrement(umax);
            if (v < vmin) v = vmin;
            if (v > vmax) v = LinAlg.BitDecrement(vmax);

            int spanU = FindSpan(p, U, u);
            int spanV = FindSpan(q, V, v);

            int iu0 = spanU - p;
            int iv0 = spanV - q;

            // Calculate basis functions and their derivatives
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

            // Calculate S_u and S_v
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

            var resultU = (Au * W - A * Wu) / (W * W);
            
            var resultV = (Av * W - A * Wv) / (W * W);

            return (resultU,resultV);
        }

        /// <summary>
        /// (en) Evaluates the second derivatives of the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェスの2階微分を評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        /// <exception cref="DivideByZeroException"></exception>
        public static (Vector3Double uu_deriv, Vector3Double uv_deriv, Vector3Double vv_deriv) EvaluateSecondDerivative(NurbsSurface surface, double u, double v)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            int p = surface.DegreeU;
            int q = surface.DegreeV;
            if (p < 0 || q < 0)
                throw new ArgumentException("Surface degrees must be non-negative.", nameof(surface));

            var cps = surface.ControlPoints;
            int nu = cps.Length;           // #CP in U
            int nv = cps[0].Length;        // #CP in V

            double[] U = surface.KnotVectorU.Knots;
            double[] V = surface.KnotVectorV.Knots;

            double umin = U[p];
            double umax = U[U.Length - p - 1];
            double vmin = V[q];
            double vmax = V[V.Length - q - 1];

            if (u < umin) u = umin;
            if (u > umax) u = LinAlg.BitDecrement(umax);
            if (v < vmin) v = vmin;
            if (v > vmax) v = LinAlg.BitDecrement(vmax);

            int spanU = FindSpan(p, U, u);
            int spanV = FindSpan(q, V, v);

            int iu0 = spanU - p;
            int iv0 = spanV - q;

            double[] Nu = new double[p + 1];
            double[] Nu_d = new double[p + 1];
            double[] Nu_d2 = new double[p + 1];
            for (int k = 0; k <= p; k++)
            {
                int i = iu0 + k;
                Nu[k] = BSplineBasisFunction(i, p, u, U);
                Nu_d[k] = DerivativeBSplineBasisFunction(i, p, u, U);
                Nu_d2[k] = DerivativeBSplineBasisFunction(i, p, u, U,2);
            }

            double[] Nv = new double[q + 1];
            double[] Nv_d = new double[q + 1];
            double[] Nv_d2 = new double[q + 1];
            for (int l = 0; l <= q; l++)
            {
                int j = iv0 + l;
                Nv[l] = BSplineBasisFunction(j, q, v, V);
                Nv_d[l] = DerivativeBSplineBasisFunction(j, q, v, V);
                Nv_d2[l] = DerivativeBSplineBasisFunction(j, q, v, V,2);
            }

            Vector3Double A = new();
            Vector3Double A_deriv_u = new(), A_deriv_v = new();
            Vector3Double A_deriv_uu = new(), A_deriv_uv = new(), A_deriv_vv=new();
            double W = 0.0;
            double W_deriv_u = 0.0, W_deriv_v = 0.0;
            double W_deriv_uu = 0.0, W_deriv_uv = 0.0, W_deriv_vv = 0.0;

            for (int k = 0; k <= p; k++)
            {
                int i = iu0 + k;
                for (int l = 0; l <= q; l++)
                {
                    int j = iv0 + l;

                    ControlPoint cp = cps[i][j];
                    double w = cp.Weight;
                    Vector3Double Pw = cp.Position * w;

                    A += Pw * Nu[k] * Nv[l];
                    W += w * Nu[k] * Nv[l];

                    A_deriv_u += Pw * Nu_d[k] * Nv[l];
                    W_deriv_u += w * Nu_d[k] * Nv[l];
                    A_deriv_v += Pw * Nu[k] * Nv_d[l];
                    W_deriv_v += w * Nu[k] * Nv_d[l];

                    A_deriv_uu += Pw * Nu_d2[k] * Nv[l];
                    W_deriv_uu += w * Nu_d2[k] * Nv[l];
                    A_deriv_uv += Pw * Nu_d[k] * Nv_d[l];
                    W_deriv_uv += w * Nu_d[k] * Nv_d[l];
                    A_deriv_vv += Pw * Nu[k] * Nv_d2[l];
                    W_deriv_vv += w * Nu[k] * Nv_d2[l];
                }
            }

            if(W == 0.0)
                throw new DivideByZeroException("The weight function evaluated to zero.");

            // Suu = (A_uu * W - 2 A_u * W_u - A * W_uu + 2 A * (W_u)^2 / W) / (W^2)
            var result_uu = (A_deriv_uu * W - 2.0 * A_deriv_u * W_deriv_u - A * W_deriv_uu + 2.0 * A * W_deriv_u * W_deriv_u / W) / (W * W);
            // Suv = (A_uv * W - A_u * W_v - A_v * W_u - A * W_uv + 2 A * W_u * W_v / W) / (W^2)
            var result_uv = (A_deriv_uv * W - A_deriv_u * W_deriv_v - A_deriv_v * W_deriv_u - A * W_deriv_uv + 2.0 * A * W_deriv_u * W_deriv_v / W) / (W * W);
            // Svv = (A_vv * W - 2 A_v * W_v - A * W_vv + 2 A * (W_v)^2 / W) / (W^2)
            var result_vv = (A_deriv_vv * W - 2.0 * A_deriv_v * W_deriv_v - A * W_deriv_vv + 2.0 * A * W_deriv_v * W_deriv_v / W) / (W * W);
            return (result_uu, result_uv, result_vv);
        }

        /// <summary>
        /// (en) Evaluates the tangent vectors and normal vector on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の接線ベクトルと法線ベクトルを評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        [Obsolete("Use Analysis.SurfaceAnalyzer.EvaluatTangentNormal instead.")]
        public static (Vector3Double tangentU,Vector3Double tangentV, Vector3Double normal) EvaluatTangentNormal(NurbsSurface surface, double u, double v)
        {
            return Analysis.SurfaceAnalyzer.EvaluatTangentNormal(surface, u, v);
        }

        /// <summary>
        /// (en) Evaluates the normal vector on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の法線ベクトルを評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        [Obsolete("Use Analysis.SurfaceAnalyzer.EvaluateNormal instead.")]
        public static Vector3Double EvaluateNormal(NurbsSurface surface, double u, double v)
        {
           return Analysis.SurfaceAnalyzer.EvaluateNormal(surface, u, v);
        }

        /// <summary>
        /// (en) Evaluates the tangent vectors on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の接線ベクトルを評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        [Obsolete("Use Analysis.SurfaceAnalyzer.EvaluateTangents instead.")]
        public static (Vector3Double tangentU, Vector3Double tangentV) EvaluateTangents(NurbsSurface surface, double u, double v)
        {
            return Analysis.SurfaceAnalyzer.EvaluateTangents(surface, u, v);
        }

        /// <summary>
        /// (en) Evaluates the principal curvatures on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の主曲率を評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="InvalidOperationException"></exception>
        [Obsolete("Use Analysis.SurfaceAnalyzer.EvaluatePrincipalCurvatures instead.")]
        public static (double k1, double k2) EvaluatePrincipalCurvatures(NurbsSurface surface, double u, double v)
        {
            return Analysis.SurfaceAnalyzer.EvaluatePrincipalCurvatures(surface, u, v);
        }

        /// <summary>
        /// (en) Evaluates the mean curvature and Gaussian curvature on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の平均曲率とガウス曲率を評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="DivideByZeroException"></exception>
        [Obsolete("Use Analysis.SurfaceAnalyzer.EvaluateMeanAndGaussianCurvatures instead.")]
        public static (double meanCurvature, double gaussianCurvature) EvaluateMeanAndGaussianCurvatures(NurbsSurface surface, double u, double v)
        {
            return Analysis.SurfaceAnalyzer.EvaluateMeanAndGaussianCurvatures(surface, u, v);
        }



    }

}
