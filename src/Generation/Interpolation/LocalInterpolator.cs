using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Generation.Interpolation
{
    /// <summary>
    /// (en) Local interpolation for NURBS curves
    /// (ja) NURBS曲線のローカル補間
    /// </summary>
    public class LocalInterpolator
    {
        /// <summary>
        /// (en) Interpolates using local method with specified knot vector
        /// (ja) 指定されたノットベクトルでローカル補間を実行
        /// </summary>
        /// <param name="points">Points to interpolate</param>
        /// <param name="degree">Curve degree</param>
        /// <param name="knotVector">Knot vector (must be compatible with points and degree)</param>
        /// <param name="options">Interpolation options (optional)</param>
        /// <returns>NURBS curve passing through all points</returns>
        /// <exception cref="ArgumentNullException">Thrown when points or knotVector is null</exception>
        /// <exception cref="ArgumentException">Thrown when parameters are incompatible</exception>
        public static NurbsCurve Interpolate(Vector3Double[] points, int degree, double[] knotVector, InterpolationOptions options = null)
        {
            Guard.ThrowIfNull(points, nameof(points));
            Guard.ThrowIfNull(knotVector, nameof(knotVector));

            if (points.Length < 2)
                throw new ArgumentException("At least 2 points are required for interpolation.", nameof(points));

            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));

            if (degree >= points.Length)
                throw new ArgumentException($"Degree ({degree}) must be less than the number of points ({points.Length}).", nameof(degree));

            int n = points.Length - 1;
            int m = n + degree + 1;

            if (knotVector.Length != m + 1)
                throw new ArgumentException($"Knot vector length ({knotVector.Length}) must equal n + p + 2 = {m + 1}.", nameof(knotVector));

            options = options ?? new InterpolationOptions();

            // 1. Parameterization - use knot-based or chord-length
            double[] u = InterpolationHelper.ComputeParameters(points, options.ParameterizationType);

            // 2. Build basis function matrix using provided knot vector
            double[][] N = InterpolationHelper.BuildBasisMatrix(n, degree, knotVector, u);

            // 3. Solve for each coordinate
            double[] Px = new double[n + 1];
            double[] Py = new double[n + 1];
            double[] Pz = new double[n + 1];

            for (int i = 0; i <= n; i++)
            {
                Px[i] = points[i].X;
                Py[i] = points[i].Y;
                Pz[i] = points[i].Z;
            }

            double[] Qx = LinAlg.SolveLinearSystem(N, Px);
            double[] Qy = LinAlg.SolveLinearSystem(N, Py);
            double[] Qz = LinAlg.SolveLinearSystem(N, Pz);

            // 4. Build control points
            ControlPoint[] controlPoints = new ControlPoint[n + 1];
            for (int i = 0; i <= n; i++)
            {
                controlPoints[i] = new ControlPoint(Qx[i], Qy[i], Qz[i], 1.0);
            }

            // 5. Create curve
            var knots = new KnotVector(knotVector, degree);
            return new NurbsCurve(degree, knots, controlPoints);
        }
    }
}
