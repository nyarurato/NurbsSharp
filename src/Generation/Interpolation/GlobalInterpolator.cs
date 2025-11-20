using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Generation.Interpolation
{
    /// <summary>
    /// (en) Global interpolation for NURBS curves
    /// (ja) NURBS曲線のグローバル補間
    /// </summary>
    public class GlobalInterpolator
    {
        /// <summary>
        /// (en) Interpolates through all points using global method
        /// (ja) グローバル法で全ての点を通る曲線を生成
        /// </summary>
        /// <param name="points">Points to interpolate</param>
        /// <param name="degree">Curve degree</param>
        /// <param name="options">Interpolation options (optional)</param>
        /// <returns>NURBS curve passing through all points</returns>
        /// <exception cref="ArgumentNullException">Thrown when points is null</exception>
        /// <exception cref="ArgumentException">Thrown when not enough points or invalid degree</exception>
        public static NurbsCurve Interpolate(Vector3Double[] points, int degree, InterpolationOptions options = null)
        {
            Guard.ThrowIfNull(points, nameof(points));
            
            if (points.Length < 2)
                throw new ArgumentException("At least 2 points are required for interpolation.", nameof(points));
            
            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));
            
            if (degree >= points.Length)
                throw new ArgumentException($"Degree ({degree}) must be less than the number of points ({points.Length}).", nameof(degree));

            options = options ?? new InterpolationOptions();

            int n = points.Length - 1;

            // 1. Parameterization
            double[] u = InterpolationHelper.ComputeParameters(points, options.ParameterizationType);

            // 2. Generate knot vector
            double[] knots = InterpolationHelper.AveragingKnotVector(n, degree, u);

            // 3. Build basis function matrix
            double[][] N = InterpolationHelper.BuildBasisMatrix(n, degree, knots, u);

            // 4. Solve for each coordinate
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

            // 5. Build control points (with unit weights for non-rational interpolation)
            ControlPoint[] controlPoints = new ControlPoint[n + 1];
            for (int i = 0; i <= n; i++)
            {
                controlPoints[i] = new ControlPoint(Qx[i], Qy[i], Qz[i], 1.0);
            }

            // 6. Create curve
            var knotVector = new KnotVector(knots, degree);
            return new NurbsCurve(degree, knotVector, controlPoints);
        }
    }
}