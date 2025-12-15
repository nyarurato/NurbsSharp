using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Analysis
{
    /// <summary>
    /// (en) Analysis utilities for NURBS surfaces
    /// (ja) NURBSサーフェスの解析ユーティリティクラス
    /// </summary>
    public class SurfaceAnalyzer: BasicAnalyzer
    {
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
        public static double SurfaceArea(NurbsSurface surface, double start_u, double end_u, double start_v, double end_v)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            if ( start_u >= end_u || start_v >= end_v)
                throw new ArgumentException("Invalid parameter range.");
            if(start_u < surface.KnotVectorU.Knots[surface.DegreeU])
                throw new ArgumentOutOfRangeException(nameof(start_u) ,"Parameter u is out of range.");
            if(end_u > surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU -1])
                throw new ArgumentOutOfRangeException(nameof(end_u) ,"Parameter u is out of range.");
            if(start_v < surface.KnotVectorV.Knots[surface.DegreeV])
                throw new ArgumentOutOfRangeException(nameof(start_v) ,"Parameter v is out of range.");
            if(end_v > surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV -1])
                throw new ArgumentOutOfRangeException(nameof(end_v) ,"Parameter v is out of range.");

            // Calculate the length of the NURBS Surface using 5-point Gaussian quadrature

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            var knotsU = surface.KnotVectorU.Knots;
            var knotsV = surface.KnotVectorV.Knots;

            if (knotsU == null || knotsU.Length < 2)
                return 0.0;
            if (knotsV == null || knotsV.Length < 2)
                return 0.0;

            // clamp integration range to valid evaluation domain

            double area = 0.0;
            
            // Detect singularities (degenerate points) at U boundaries
            const double singularityTolerance = 1e-10;
            bool hasStartUSingularity = IsDegenerateRow(surface, knotsU[degreeU], singularityTolerance);
            bool hasEndUSingularity = IsDegenerateRow(surface, knotsU[knotsU.Length - degreeU - 1], singularityTolerance);

            // integrate over each knot span to better capture local behavior
            for (int i = 0; i < knotsU.Length - 1; i++)
            {
                double a_u = Math.Max(start_u, knotsU[i]);
                double b_u = Math.Min(end_u, knotsU[i + 1]);

                if (b_u <= a_u)
                    continue;
                
                // Check if this span is at a singular boundary
                bool isAtStartSingularity = hasStartUSingularity && Math.Abs(a_u - knotsU[degreeU]) < singularityTolerance;
                bool isAtEndSingularity = hasEndUSingularity && Math.Abs(b_u - knotsU[knotsU.Length - degreeU - 1]) < singularityTolerance;
                
                // Skip small epsilon region near singularities to avoid numerical issues
                if (isAtStartSingularity)
                {
                    double epsilon = Math.Min(1e-6, (b_u - a_u) * 0.01);
                    a_u += epsilon;
                    if (b_u <= a_u) continue;
                }
                if (isAtEndSingularity)
                {
                    double epsilon = Math.Min(1e-6, (b_u - a_u) * 0.01);
                    b_u -= epsilon;
                    if (b_u <= a_u) continue;
                }

                double half_u = 0.5 * (b_u - a_u);
                double center_u = 0.5 * (a_u + b_u);

                for (int j = 0; j < knotsV.Length - 1; j++)
                {
                    double a_v = Math.Max(start_v, knotsV[j]);
                    double b_v = Math.Min(end_v, knotsV[j + 1]);
                    if (b_v <= a_v)
                        continue;
                    double half_v = 0.5 * (b_v - a_v);
                    double center_v = 0.5 * (a_v + b_v);
                    
                    double spanSum = 0.0;
                    
                    // double Gauss loop: u-nodes and v-nodes
                    for (int iu = 0; iu < GaussNode5.Length; iu++)
                    {
                        double xi_u = GaussNode5[iu];
                        double wi_u = GaussWeight5[iu];
                        double u = center_u + half_u * xi_u;// map from [-1,1] to [a_u,b_u]

                        for (int iv = 0; iv < GaussNode5.Length; iv++)
                        {
                            double xi_v = GaussNode5[iv];
                            double wi_v = GaussWeight5[iv];
                            double v = center_v + half_v * xi_v;// map from [-1,1] to [a_v,b_v]

                            (Vector3Double dU, Vector3Double dV) = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                            double dA = Vector3Double.Cross(dU, dV).magnitude;
                            spanSum += wi_u * wi_v * dA;
                        }
                    }

                    // multiply by Jacobian of the mapping from [-1,1]^2 -> [a_u,b_u]x[a_v,b_v]
                    area += half_u * half_v * spanSum;
                }
            }

            return area;
        }
        
        /// <summary>
        /// (en)Check if a row at parameter u is degenerate (all control points at same position)
        /// (ja) パラメータ u での行が縮退しているかどうかを確認します（すべての制御点が同じ位置にある）
        /// </summary>
        private static bool IsDegenerateRow(NurbsSurface surface, double u, double tolerance)
        {
            var firstPoint = SurfaceEvaluator.Evaluate(surface, u, 0.0);
            var lastPoint = SurfaceEvaluator.Evaluate(surface, u, 1.0);
            return firstPoint.DistanceTo(lastPoint) < tolerance;
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
        public static (Vector3Double tangentU,Vector3Double tangentV, Vector3Double normal) EvaluatTangentNormal(NurbsSurface surface, double u, double v)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            (Vector3Double dU, Vector3Double dV) = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);

            //tangent 
            Vector3Double tangentU = dU.normalized;
            Vector3Double tangentV = dV.normalized;

            // normal = dU x dV
            Vector3Double normal = Vector3Double.Cross(dU, dV).normalized;

            return (tangentU, tangentV, normal);
        }

        /// <summary>
        /// (en) Evaluates the normal vector on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の法線ベクトルを評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector3Double EvaluateNormal(NurbsSurface surface, double u, double v)
        {
           return EvaluatTangentNormal(surface, u, v).normal;
        }

        /// <summary>
        /// (en) Evaluates the tangent vectors on the NURBS surface at the specified parameters u and v.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の接線ベクトルを評価します。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public static (Vector3Double tangentU, Vector3Double tangentV) EvaluateTangents(NurbsSurface surface, double u, double v)
        {
            var (tangentU, tangentV, normal) = EvaluatTangentNormal(surface, u, v);
            return (tangentU,tangentV);
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
        public static (double k1, double k2) EvaluatePrincipalCurvatures(NurbsSurface surface, double u, double v)
        {
            // Principal curvatures k1 , k2 => det(II - k * I) = 0

            // Get first derivatives
            (Vector3Double Su, Vector3Double Sv) = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
            // Get second derivatives
            (Vector3Double Suu, Vector3Double Suv, Vector3Double Svv) = SurfaceEvaluator.EvaluateSecondDerivative(surface, u, v);
            // Normal vector
            Vector3Double Normal = Vector3Double.Cross(Su, Sv).normalized;
            // First fundamental form coefficients
            double E = Vector3Double.Dot(Su, Su);
            double F = Vector3Double.Dot(Su, Sv);
            double G = Vector3Double.Dot(Sv, Sv);
            // Second fundamental form coefficients
            double L = Vector3Double.Dot(Suu, Normal);
            double M = Vector3Double.Dot(Suv, Normal);
            double N = Vector3Double.Dot(Svv, Normal);

            double a = (E * G - F * F);
            double b = (E * N - 2 * F * M + G * L);
            double c = (L * N - M * M);

            double discriminant = b * b - 4 * a * c; //sqrt(b^2 - 4ac)
            if (discriminant < 0)
            {
                throw new InvalidOperationException("The principal curvatures are complex.");
            }
            // Compute the principal curvatures
            double sqrtDiscriminant = Math.Sqrt(discriminant);
            double k1 = (b + sqrtDiscriminant) / (2 * a);
            double k2 = (b - sqrtDiscriminant) / (2 * a);
            return (k1, k2);
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
        public static (double meanCurvature, double gaussianCurvature) EvaluateMeanAndGaussianCurvatures(NurbsSurface surface, double u, double v)
        {
            // Gaussian curvature K = det(II) / det(I) = (L*N - M^2) / (E*G - F^2)
            // Mean curvature H = trace(II * I^-1) / 2 = (E*N - 2*F*M + G*L) / (2*(E*G - F^2))

            // Get first derivatives
            (Vector3Double Su, Vector3Double Sv) = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
            // Get second derivatives
            (Vector3Double Suu, Vector3Double Suv, Vector3Double Svv) = SurfaceEvaluator.EvaluateSecondDerivative(surface, u, v);
            // Normal vector
            Vector3Double Normal = Vector3Double.Cross(Su, Sv).normalized;
            // First fundamental form coefficients
            double E = Vector3Double.Dot(Su, Su);
            double F = Vector3Double.Dot(Su, Sv);
            double G = Vector3Double.Dot(Sv, Sv);
            // Second fundamental form coefficients
            double L = Vector3Double.Dot(Suu, Normal);
            double M = Vector3Double.Dot(Suv, Normal);
            double N = Vector3Double.Dot(Svv, Normal);
            // Compute mean curvature H and Gaussian curvature K
            double denominator = E * G - F * F;
            if (denominator == 0)
            {
                throw new DivideByZeroException("Denominator in curvature calculation is zero.");
            }
            double meanCurvature = (E * N - 2 * F * M + G * L) / (2 * denominator); 
            double gaussianCurvature = (L * N - M * M) / denominator;
            return (meanCurvature, gaussianCurvature);
        }
    }
}