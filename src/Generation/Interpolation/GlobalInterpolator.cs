using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Generation.Interpolation
{
    /// <summary>
    /// (en) Global interpolation for NURBS curves and surfaces
    /// (ja) NURBS曲線とサーフェスのグローバル補間
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
        public static NurbsCurve InterpolateCurve(Vector3Double[] points, int degree, InterpolationOptions? options = null)
        {
            Guard.ThrowIfNull(points, nameof(points));
            
            if (points.Length < 2)
                throw new ArgumentException("At least 2 points are required for interpolation.", nameof(points));
            
            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));
            
            if (degree >= points.Length)
                throw new ArgumentException($"Degree ({degree}) must be less than the number of points ({points.Length}).", nameof(degree));

            options ??= new InterpolationOptions();

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

        /// <summary>
        /// (en) Interpolates through a grid of points using global method
        /// (ja) グローバル法で点グリッドを通るサーフェスを生成
        /// </summary>
        /// <param name="points">Points grid to interpolate [U][V]</param>
        /// <param name="degreeU">Surface degree in U direction</param>
        /// <param name="degreeV">Surface degree in V direction</param>
        /// <param name="options">Interpolation options (optional)</param>
        /// <returns>NURBS surface passing through all points</returns>
        /// <exception cref="ArgumentNullException">Thrown when points is null</exception>
        /// <exception cref="ArgumentException">Thrown when invalid input</exception>
        public static NurbsSurface InterpolateSurf(Vector3Double[][] points, int degreeU, int degreeV, InterpolationOptions? options = null)
        {
            // 1. Validate inputs
            Guard.ThrowIfNull(points, nameof(points));

            if (points.Length < 2)
                throw new ArgumentException("At least 2 rows of points are required for surface interpolation.", nameof(points));

            int size_u = points.Length;
            int size_v = points[0].Length;

            if (size_v < 2)
                throw new ArgumentException("At least 2 columns of points are required for surface interpolation.", nameof(points));

            // Validate rectangular grid
            for (int i = 1; i < size_u; i++)
            {
                if (points[i].Length != size_v)
                    throw new ArgumentException("Points must form a rectangular grid. All rows must have the same number of columns.", nameof(points));
            }

            if (degreeU < 1)
                throw new ArgumentException("Degree U must be at least 1.", nameof(degreeU));

            if (degreeV < 1)
                throw new ArgumentException("Degree V must be at least 1.", nameof(degreeV));

            if (degreeU >= size_u)
                throw new ArgumentException($"Degree U ({degreeU}) must be less than the number of U points ({size_u}).", nameof(degreeU));

            if (degreeV >= size_v)
                throw new ArgumentException($"Degree V ({degreeV}) must be less than the number of V points ({size_v}).", nameof(degreeV));

            options ??= new InterpolationOptions();

            // 2. Compute parameters
            var (uk, vl) = InterpolationHelper.ComputeParametersSurface(points, options.ParameterizationType);

            // 3. Generate knot vectors
            double[] knotsU = InterpolationHelper.AveragingKnotVector(size_u - 1, degreeU, uk);
            double[] knotsV = InterpolationHelper.AveragingKnotVector(size_v - 1, degreeV, vl);

            // 4. Build basis matrices (reusable for all rows/columns)
            double[][] Nu = InterpolationHelper.BuildBasisMatrix(size_u - 1, degreeU, knotsU, uk);
            double[][] Nv = InterpolationHelper.BuildBasisMatrix(size_v - 1, degreeV, knotsV, vl);

            // 5. First pass: Interpolate in U direction for each V column
            ControlPoint[][] R = new ControlPoint[size_v][];

            for (int v = 0; v < size_v; v++)
            {
                // Extract points along U direction for this V column
                Vector3Double[] ptsU = new Vector3Double[size_u];
                for (int u = 0; u < size_u; u++)
                {
                    ptsU[u] = points[u][v];
                }

                // Separate coordinates
                double[] Px = new double[size_u];
                double[] Py = new double[size_u];
                double[] Pz = new double[size_u];

                for (int i = 0; i < size_u; i++)
                {
                    Px[i] = ptsU[i].X;
                    Py[i] = ptsU[i].Y;
                    Pz[i] = ptsU[i].Z;
                }

                // Solve for control points in U direction
                double[] Qx = LinAlg.SolveLinearSystem(Nu, Px);
                double[] Qy = LinAlg.SolveLinearSystem(Nu, Py);
                double[] Qz = LinAlg.SolveLinearSystem(Nu, Pz);

                // Store intermediate results
                R[v] = new ControlPoint[size_u];
                for (int u = 0; u < size_u; u++)
                {
                    R[v][u] = new ControlPoint(Qx[u], Qy[u], Qz[u], 1.0);
                }
            }

            // 6. Second pass: Interpolate in V direction for each U row
            ControlPoint[][] controlPoints = new ControlPoint[size_u][];

            for (int u = 0; u < size_u; u++)
            {
                // Extract intermediate control points along V direction for this U row
                double[] Rx = new double[size_v];
                double[] Ry = new double[size_v];
                double[] Rz = new double[size_v];

                for (int v = 0; v < size_v; v++)
                {
                    Rx[v] = R[v][u].Position.X;
                    Ry[v] = R[v][u].Position.Y;
                    Rz[v] = R[v][u].Position.Z;
                }

                // Solve for final control points in V direction
                double[] Qx = LinAlg.SolveLinearSystem(Nv, Rx);
                double[] Qy = LinAlg.SolveLinearSystem(Nv, Ry);
                double[] Qz = LinAlg.SolveLinearSystem(Nv, Rz);

                // Store final control points
                controlPoints[u] = new ControlPoint[size_v];
                for (int v = 0; v < size_v; v++)
                {
                    controlPoints[u][v] = new ControlPoint(Qx[v], Qy[v], Qz[v], 1.0);
                }
            }

            // 7. Create surface
            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);
            return new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
        }
    }
}