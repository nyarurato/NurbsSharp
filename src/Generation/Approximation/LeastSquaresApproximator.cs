using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;
using NurbsSharp.Generation.Interpolation;

namespace NurbsSharp.Generation.Approximation
{
    /// <summary>
    /// (en) Least squares approximation for NURBS curves and surfaces
    /// (ja) NURBS曲線とサーフェスの最小二乗近似
    /// </summary>
    public class LeastSquaresApproximator
    {
        /// <summary>
        /// (en) Approximates data points with fewer control points using least squares method
        /// (ja) 最小二乗法でデータ点を少ない制御点で近似
        /// </summary>
        /// <param name="points">Data points to approximate</param>
        /// <param name="degree">Curve degree</param>
        /// <param name="numControlPoints">Number of control points (must be less than number of data points)</param>
        /// <param name="options">Approximation options (optional)</param>
        /// <returns>NURBS curve approximating the data points</returns>
        /// <exception cref="ArgumentNullException">Thrown when points is null</exception>
        /// <exception cref="ArgumentException">Thrown when invalid parameters</exception>
        public static NurbsCurve ApproximateCurve(Vector3Double[] points, int degree, int numControlPoints, ApproximationOptions? options = null)
        {
            Guard.ThrowIfNull(points, nameof(points));

            int n = points.Length - 1; // n+1 data points (index 0 to n)
            int m = numControlPoints - 1; // m+1 control points (index 0 to m)

            if (points.Length < 2)
                throw new ArgumentException("At least 2 points are required for approximation.", nameof(points));

            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));

            if (numControlPoints <= degree)
                throw new ArgumentException($"Number of control points ({numControlPoints}) must be greater than degree ({degree}).", nameof(numControlPoints));

            if (numControlPoints > points.Length)
                throw new ArgumentException($"Number of control points ({numControlPoints}) must be less than or equal to number of data points ({points.Length}).", nameof(numControlPoints));

            options ??= new ApproximationOptions();

            // 1. Compute parameters for data points
            double[] uk = InterpolationHelper.ComputeParameters(points, options.ParameterizationType);

            // 2. Compute knot vector using averaging method
            // For approximation with m+1 control points and n+1 data points
            double[] U = ComputeApproximationKnotVector(m, degree, n, uk);

            // 3. Build the least squares system
            // If clamping ends: P0 = Q0 and Pm = Qn (first and last control points fixed)
            // Solve for interior control points P1, ..., Pm-1

            ControlPoint[] controlPoints;

            if (options.ClampEnds)
            {
                // Fix first and last control points to first and last data points
                controlPoints = ApproximateWithClampedEnds(points, degree, m, uk, U);
            }
            else
            {
                // All control points are free
                controlPoints = ApproximateWithFreeEnds(points, degree, m, uk, U);
            }

            // 4. Create curve
            var knotVector = new KnotVector(U, degree);
            return new NurbsCurve(degree, knotVector, controlPoints);
        }

        /// <summary>
        /// (en) Approximates a grid of points with fewer control points using least squares method
        /// (ja) 最小二乗法で点グリッドを少ない制御点で近似
        /// </summary>
        /// <param name="points">Data points grid [U][V]</param>
        /// <param name="degreeU">Surface degree in U direction</param>
        /// <param name="degreeV">Surface degree in V direction</param>
        /// <param name="numControlPointsU">Number of control points in U direction</param>
        /// <param name="numControlPointsV">Number of control points in V direction</param>
        /// <param name="options">Approximation options (optional)</param>
        /// <returns>NURBS surface approximating the data points</returns>
        /// <exception cref="ArgumentNullException">Thrown when points is null</exception>
        /// <exception cref="ArgumentException">Thrown when invalid parameters</exception>
        public static NurbsSurface ApproximateSurface(Vector3Double[][] points, int degreeU, int degreeV, 
            int numControlPointsU, int numControlPointsV, ApproximationOptions? options = null)
        {
            // 1. Validate inputs
            Guard.ThrowIfNull(points, nameof(points));

            if (points.Length < 2)
                throw new ArgumentException("At least 2 rows of points are required.", nameof(points));

            int sizeU = points.Length;
            int sizeV = points[0].Length;

            if (sizeV < 2)
                throw new ArgumentException("At least 2 columns of points are required.", nameof(points));

            // Validate rectangular grid
            for (int i = 1; i < sizeU; i++)
            {
                if (points[i].Length != sizeV)
                    throw new ArgumentException("Points must form a rectangular grid.", nameof(points));
            }

            if (degreeU < 1 || degreeV < 1)
                throw new ArgumentException("Degrees must be at least 1.");

            if (numControlPointsU <= degreeU)
                throw new ArgumentException($"Number of U control points ({numControlPointsU}) must be greater than degree U ({degreeU}).");

            if (numControlPointsV <= degreeV)
                throw new ArgumentException($"Number of V control points ({numControlPointsV}) must be greater than degree V ({degreeV}).");

            if (numControlPointsU > sizeU)
                throw new ArgumentException($"Number of U control points ({numControlPointsU}) must be <= number of U data points ({sizeU}).");

            if (numControlPointsV > sizeV)
                throw new ArgumentException($"Number of V control points ({numControlPointsV}) must be <= number of V data points ({sizeV}).");

            options ??= new ApproximationOptions();

            int n = sizeU - 1;
            int m = sizeV - 1;
            int p = numControlPointsU - 1;
            int q = numControlPointsV - 1;

            // 2. Compute parameters
            var (uk, vl) = InterpolationHelper.ComputeParametersSurface(points, options.ParameterizationType);

            // 3. Generate knot vectors
            double[] knotsU = ComputeApproximationKnotVector(p, degreeU, n, uk);
            double[] knotsV = ComputeApproximationKnotVector(q, degreeV, m, vl);

            // 4. Approximate in two passes (similar to interpolation but with least squares)
            ControlPoint[][] R;

            if (options.ClampEnds)
            {
                R = ApproximateSurfaceFirstPassClamped(points, degreeU, p, n, m, uk, knotsU);
            }
            else
            {
                R = ApproximateSurfaceFirstPassFree(points, degreeU, p, n, m, uk, knotsU);
            }

            // 5. Second pass: Approximate in V direction
            ControlPoint[][] controlPoints;

            if (options.ClampEnds)
            {
                controlPoints = ApproximateSurfaceSecondPassClamped(R, degreeV, q, p, m, vl, knotsV);
            }
            else
            {
                controlPoints = ApproximateSurfaceSecondPassFree(R, degreeV, q, p, m, vl, knotsV);
            }

            // 6. Create surface
            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);
            return new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
        }


        /// <summary>
        /// Compute knot vector for approximation (NURBS Book Eq. 9.68-9.69)
        /// </summary>
        private static double[] ComputeApproximationKnotVector(int m, int p, int n, double[] uk)
        {
            int knotCount = m + p + 2;
            double[] U = new double[knotCount];

            // Clamp the first and last knots
            for (int i = 0; i <= p; i++)
            {
                U[i] = 0.0;
                U[knotCount - 1 - i] = 1.0;
            }

            // Compute interior knots based on data point parameters
            // d = (n + 1) / (m - p + 1) - spacing between knot influence regions
            double d = (double)(n + 1) / (m - p + 1);

            for (int j = 1; j <= m - p; j++)
            {
                int i = (int)Math.Floor(j * d);
                double alpha = j * d - i;
                U[p + j] = (1.0 - alpha) * uk[i - 1] + alpha * uk[i];
            }

            return U;
        }

        /// <summary>
        /// Approximate with clamped ends (NURBS Book Algorithm A9.7)
        /// </summary>
        private static ControlPoint[] ApproximateWithClampedEnds(Vector3Double[] points, int p, int m, double[] uk, double[] U)
        {
            int n = points.Length - 1;
            ControlPoint[] controlPoints = new ControlPoint[m + 1];

            // Fix first and last control points
            controlPoints[0] = new ControlPoint(points[0], 1.0);
            controlPoints[m] = new ControlPoint(points[n], 1.0);

            // If only 2 control points, we're done
            if (m == 1) return controlPoints;

            // Build least squares system for interior control points
            // Solve N^T * N * P = N^T * R where R = Q - N_0*P_0 - N_m*P_m

            int numInterior = m - 1; // Number of unknowns (P_1 to P_{m-1})

            // Build N^T * N matrix (symmetric, positive definite)
            double[][] NTN = new double[numInterior][];
            for (int i = 0; i < numInterior; i++)
            {
                NTN[i] = new double[numInterior];
            }

            // Build N^T * R vectors for x, y, z
            double[] NTRx = new double[numInterior];
            double[] NTRy = new double[numInterior];
            double[] NTRz = new double[numInterior];

            // For each data point
            for (int k = 1; k < n; k++) // Skip first and last (they're fixed)
            {
                // Compute basis functions at uk[k]
                double[] N = new double[m + 1];
                for (int i = 0; i <= m; i++)
                {
                    N[i] = BasicEvaluator.BSplineBasisFunction(i, p, uk[k], U);
                }

                // Contribution to right-hand side (subtract fixed endpoints)
                Vector3Double Rk = points[k] - (N[0] * points[0] + N[m] * points[n]);

                // Build N^T * N and N^T * R
                for (int i = 0; i < numInterior; i++)
                {
                    int ii = i + 1; // Map to actual control point index
                    double Ni = N[ii];

                    NTRx[i] += Ni * Rk.X;
                    NTRy[i] += Ni * Rk.Y;
                    NTRz[i] += Ni * Rk.Z;

                    for (int j = 0; j < numInterior; j++)
                    {
                        int jj = j + 1;
                        NTN[i][j] += Ni * N[jj];
                    }
                }
            }

            // Solve for interior control points
            double[] Px = LinAlg.SolveLinearSystem(NTN, NTRx);
            double[] Py = LinAlg.SolveLinearSystem(NTN, NTRy);
            double[] Pz = LinAlg.SolveLinearSystem(NTN, NTRz);

            for (int i = 0; i < numInterior; i++)
            {
                controlPoints[i + 1] = new ControlPoint(Px[i], Py[i], Pz[i], 1.0);
            }

            return controlPoints;
        }

        /// <summary>
        /// Approximate with free ends
        /// </summary>
        private static ControlPoint[] ApproximateWithFreeEnds(Vector3Double[] points, int p, int m, double[] uk, double[] U)
        {
            int n = points.Length - 1;

            // Build least squares system for all control points
            double[][] NTN = new double[m + 1][];
            for (int i = 0; i <= m; i++)
            {
                NTN[i] = new double[m + 1];
            }

            double[] NTQx = new double[m + 1];
            double[] NTQy = new double[m + 1];
            double[] NTQz = new double[m + 1];

            // For each data point
            for (int k = 0; k <= n; k++)
            {
                // Compute basis functions at uk[k]
                double[] N = new double[m + 1];
                for (int i = 0; i <= m; i++)
                {
                    N[i] = BasicEvaluator.BSplineBasisFunction(i, p, uk[k], U);
                }

                // Build N^T * N and N^T * Q
                for (int i = 0; i <= m; i++)
                {
                    NTQx[i] += N[i] * points[k].X;
                    NTQy[i] += N[i] * points[k].Y;
                    NTQz[i] += N[i] * points[k].Z;

                    for (int j = 0; j <= m; j++)
                    {
                        NTN[i][j] += N[i] * N[j];
                    }
                }
            }

            // Solve for control points
            double[] Px = LinAlg.SolveLinearSystem(NTN, NTQx);
            double[] Py = LinAlg.SolveLinearSystem(NTN, NTQy);
            double[] Pz = LinAlg.SolveLinearSystem(NTN, NTQz);

            ControlPoint[] controlPoints = new ControlPoint[m + 1];
            for (int i = 0; i <= m; i++)
            {
                controlPoints[i] = new ControlPoint(Px[i], Py[i], Pz[i], 1.0);
            }

            return controlPoints;
        }


        private static ControlPoint[][] ApproximateSurfaceFirstPassClamped(
            Vector3Double[][] points, int p, int numCtrlU, int numDataU, int numDataV, 
            double[] uk, double[] knotsU)
        {
            ControlPoint[][] R = new ControlPoint[numDataV + 1][];

            for (int l = 0; l <= numDataV; l++)
            {
                // Extract points along U direction for this V column
                Vector3Double[] uPoints = new Vector3Double[numDataU + 1];
                for (int k = 0; k <= numDataU; k++)
                {
                    uPoints[k] = points[k][l];
                }

                // Approximate this column
                R[l] = ApproximateWithClampedEnds(uPoints, p, numCtrlU, uk, knotsU);
            }

            return R;
        }

        private static ControlPoint[][] ApproximateSurfaceFirstPassFree(
            Vector3Double[][] points, int p, int numCtrlU, int numDataU, int numDataV, 
            double[] uk, double[] knotsU)
        {
            ControlPoint[][] R = new ControlPoint[numDataV + 1][];

            for (int l = 0; l <= numDataV; l++)
            {
                Vector3Double[] uPoints = new Vector3Double[numDataU + 1];
                for (int k = 0; k <= numDataU; k++)
                {
                    uPoints[k] = points[k][l];
                }

                R[l] = ApproximateWithFreeEnds(uPoints, p, numCtrlU, uk, knotsU);
            }

            return R;
        }

        private static ControlPoint[][] ApproximateSurfaceSecondPassClamped(
            ControlPoint[][] R, int q, int numCtrlV, int numCtrlU, int numDataV, 
            double[] vl, double[] knotsV)
        {
            ControlPoint[][] controlPoints = new ControlPoint[numCtrlU + 1][];

            for (int k = 0; k <= numCtrlU; k++)
            {
                // Extract intermediate control points along V direction
                Vector3Double[] vPoints = new Vector3Double[numDataV + 1];
                for (int l = 0; l <= numDataV; l++)
                {
                    vPoints[l] = R[l][k].Position;
                }

                // Approximate this row
                controlPoints[k] = ApproximateWithClampedEnds(vPoints, q, numCtrlV, vl, knotsV);
            }

            return controlPoints;
        }

        private static ControlPoint[][] ApproximateSurfaceSecondPassFree(
            ControlPoint[][] R, int q, int numCtrlV, int numCtrlU, int numDataV, 
            double[] vl, double[] knotsV)
        {
            ControlPoint[][] controlPoints = new ControlPoint[numCtrlU + 1][];

            for (int k = 0; k <= numCtrlU; k++)
            {
                Vector3Double[] vPoints = new Vector3Double[numDataV + 1];
                for (int l = 0; l <= numDataV; l++)
                {
                    vPoints[l] = R[l][k].Position;
                }

                controlPoints[k] = ApproximateWithFreeEnds(vPoints, q, numCtrlV, vl, knotsV);
            }

            return controlPoints;
        }
    }
}
