using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Generation.Approximation;
using NurbsSharp.Generation.Interpolation;
using System;

namespace UnitTests.Generation
{
    [TestFixture]
    public class ApproximationTest
    {
        [Test]
        public void ApproximateCurve_ClampedEnds_ReducesControlPoints()
        {
            // Arrange: Create 10 data points on a smooth curve
            var dataPoints = new Vector3Double[10];
            for (int i = 0; i < 10; i++)
            {
                double t = i / 9.0;
                dataPoints[i] = new Vector3Double(t, Math.Sin(t * Math.PI), 0);
            }

            int degree = 3;
            int numControlPoints = 6; // Fewer than data points

            // Act
            var curve = LeastSquaresApproximator.ApproximateCurve(
                dataPoints, degree, numControlPoints, 
                new ApproximationOptions { ClampEnds = true });

            // Assert
            using (Assert.EnterMultipleScope())
            {
                Assert.That(curve, Is.Not.Null);
                Assert.That(curve.Degree, Is.EqualTo(degree));
                Assert.That(curve.ControlPoints.Length, Is.EqualTo(numControlPoints));
            }

            // Verify endpoints are fixed
            var start = curve.GetPos(0.0);
            var end = curve.GetPos(1.0);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(start.X, Is.EqualTo(dataPoints[0].X).Within(1e-10));
                Assert.That(start.Y, Is.EqualTo(dataPoints[0].Y).Within(1e-10));
                Assert.That(end.X, Is.EqualTo(dataPoints[9].X).Within(1e-10));
                Assert.That(end.Y, Is.EqualTo(dataPoints[9].Y).Within(1e-10));
            }
        }

        [Test]
        public void ApproximateCurve_FreeEnds_AllControlPointsFree()
        {
            // Arrange
            var dataPoints = new Vector3Double[8];
            for (int i = 0; i < 8; i++)
            {
                double t = i / 7.0;
                dataPoints[i] = new Vector3Double(t, t * t, 0);
            }

            int degree = 2;
            int numControlPoints = 5;

            // Act
            var curve = LeastSquaresApproximator.ApproximateCurve(
                dataPoints, degree, numControlPoints,
                new ApproximationOptions { ClampEnds = false });

            // Assert
            using (Assert.EnterMultipleScope())
            {
                Assert.That(curve, Is.Not.Null);
                Assert.That(curve.Degree, Is.EqualTo(degree));
                Assert.That(curve.ControlPoints.Length, Is.EqualTo(numControlPoints));
            }

            // With free ends, the curve approximates but may not pass through endpoints
            var start = curve.GetPos(0.0);
            var end = curve.GetPos(1.0);

            // Should be close but not necessarily exact
            Assert.That(start.DistanceTo(dataPoints[0]), Is.LessThan(0.2));
            Assert.That(end.DistanceTo(dataPoints[7]), Is.LessThan(0.2));
        }

        [Test]
        public void ApproximateCurve_DifferentParameterizations()
        {
            // Arrange
            var dataPoints = new Vector3Double[12];
            for (int i = 0; i < 12; i++)
            {
                double angle = i / 11.0 * 2 * Math.PI;
                dataPoints[i] = new Vector3Double(Math.Cos(angle), Math.Sin(angle), 0);
            }

            int degree = 3;
            int numControlPoints = 8;

            // Act - Test different parameterizations
            var curveChord = LeastSquaresApproximator.ApproximateCurve(
                dataPoints, degree, numControlPoints,
                new ApproximationOptions { ParameterizationType = ParameterizationType.Chord });

            var curveCentripetal = LeastSquaresApproximator.ApproximateCurve(
                dataPoints, degree, numControlPoints,
                new ApproximationOptions { ParameterizationType = ParameterizationType.Centripetal });

            var curveUniform = LeastSquaresApproximator.ApproximateCurve(
                dataPoints, degree, numControlPoints,
                new ApproximationOptions { ParameterizationType = ParameterizationType.Uniform });

            // Assert - All should produce valid curves
            Assert.That(curveChord.ControlPoints.Length, Is.EqualTo(numControlPoints));
            Assert.That(curveCentripetal.ControlPoints.Length, Is.EqualTo(numControlPoints));
            Assert.That(curveUniform.ControlPoints.Length, Is.EqualTo(numControlPoints));
        }

        [Test]
        public void ApproximateCurve_ComputesApproximationError()
        {
            // Arrange: Create noisy data
            var dataPoints = new Vector3Double[20];
            var random = new Random(42);
            for (int i = 0; i < 20; i++)
            {
                double t = i / 19.0;
                double noise = (random.NextDouble() - 0.5) * 0.05;
                dataPoints[i] = new Vector3Double(t, Math.Sin(t * 2 * Math.PI) + noise, 0);
            }

            int degree = 3;
            int numControlPoints = 8;

            // Act
            var curve = LeastSquaresApproximator.ApproximateCurve(
                dataPoints, degree, numControlPoints);

            // Assert - Compute approximation error
            double[] parameters = InterpolationHelper.ComputeParameters(dataPoints, ParameterizationType.Chord);
            double maxError = 0.0;
            double sumSquaredError = 0.0;

            for (int i = 0; i < dataPoints.Length; i++)
            {
                var evaluated = curve.GetPos(parameters[i]);
                double error = evaluated.DistanceTo(dataPoints[i]);
                maxError = Math.Max(maxError, error);
                sumSquaredError += error * error;
            }

            double rmsError = Math.Sqrt(sumSquaredError / dataPoints.Length);

            // With approximation, errors should be small but non-zero
            using (Assert.EnterMultipleScope())
            {
                Assert.That(maxError, Is.LessThan(0.2), "Max error should be reasonable");
                Assert.That(rmsError, Is.LessThan(0.1), "RMS error should be small");
            }
        }

        [Test]
        public void ApproximateSurface_ClampedEnds_ReducesControlPoints()
        {
            // Arrange: Create a grid of data points (8x6)
            int dataU = 8, dataV = 6;
            var dataPoints = new Vector3Double[dataU][];

            for (int u = 0; u < dataU; u++)
            {
                dataPoints[u] = new Vector3Double[dataV];
                for (int v = 0; v < dataV; v++)
                {
                    double x = u / (dataU - 1.0);
                    double y = v / (dataV - 1.0);
                    double z = Math.Sin(x * Math.PI) * Math.Cos(y * Math.PI);
                    dataPoints[u][v] = new Vector3Double(x, y, z);
                }
            }

            int degreeU = 3, degreeV = 3;
            int numCtrlU = 5, numCtrlV = 4; // Fewer than data points

            // Act
            var surface = LeastSquaresApproximator.ApproximateSurface(
                dataPoints, degreeU, degreeV, numCtrlU, numCtrlV,
                new ApproximationOptions { ClampEnds = true });

            // Assert
            using (Assert.EnterMultipleScope())
            {
                Assert.That(surface, Is.Not.Null);
                Assert.That(surface.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface.ControlPoints.Length, Is.EqualTo(numCtrlU));
                Assert.That(surface.ControlPoints[0].Length, Is.EqualTo(numCtrlV));
            }

            // Verify corners are fixed
            var corner00 = surface.GetPos(0.0, 0.0);
            var corner10 = surface.GetPos(1.0, 0.0);
            var corner01 = surface.GetPos(0.0, 1.0);
            var corner11 = surface.GetPos(1.0, 1.0);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(corner00.DistanceTo(dataPoints[0][0]), Is.LessThan(1e-8));
                Assert.That(corner10.DistanceTo(dataPoints[dataU - 1][0]), Is.LessThan(1e-8));
                Assert.That(corner01.DistanceTo(dataPoints[0][dataV - 1]), Is.LessThan(1e-8));
                Assert.That(corner11.DistanceTo(dataPoints[dataU - 1][dataV - 1]), Is.LessThan(1e-8));
            }
        }

        [Test]
        public void ApproximateSurface_FreeEnds_AllControlPointsFree()
        {
            // Arrange
            int dataU = 6, dataV = 5;
            var dataPoints = new Vector3Double[dataU][];

            for (int u = 0; u < dataU; u++)
            {
                dataPoints[u] = new Vector3Double[dataV];
                for (int v = 0; v < dataV; v++)
                {
                    double x = u / (dataU - 1.0);
                    double y = v / (dataV - 1.0);
                    double z = x * x + y * y;
                    dataPoints[u][v] = new Vector3Double(x, y, z);
                }
            }

            int degreeU = 2, degreeV = 2;
            int numCtrlU = 4, numCtrlV = 3;

            // Act
            var surface = LeastSquaresApproximator.ApproximateSurface(
                dataPoints, degreeU, degreeV, numCtrlU, numCtrlV,
                new ApproximationOptions { ClampEnds = false });

            // Assert
            using (Assert.EnterMultipleScope())
            {
                Assert.That(surface, Is.Not.Null);
                Assert.That(surface.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface.ControlPoints.Length, Is.EqualTo(numCtrlU));
                Assert.That(surface.ControlPoints[0].Length, Is.EqualTo(numCtrlV));
            }
        }

        [Test]
        public void ApproximateSurface_ComputesApproximationError()
        {
            // Arrange: Create a simple saddle surface with noise
            int dataU = 10, dataV = 8;
            var dataPoints = new Vector3Double[dataU][];
            var random = new Random(123);

            for (int u = 0; u < dataU; u++)
            {
                dataPoints[u] = new Vector3Double[dataV];
                for (int v = 0; v < dataV; v++)
                {
                    double x = u / (dataU - 1.0) * 2 - 1;
                    double y = v / (dataV - 1.0) * 2 - 1;
                    double noise = (random.NextDouble() - 0.5) * 0.02;
                    double z = x * x - y * y + noise;
                    dataPoints[u][v] = new Vector3Double(x, y, z);
                }
            }

            int degreeU = 3, degreeV = 3;
            int numCtrlU = 6, numCtrlV = 5;

            // Act
            var surface = LeastSquaresApproximator.ApproximateSurface(
                dataPoints, degreeU, degreeV, numCtrlU, numCtrlV);

            // Assert - Compute approximation error
            var (uk, vl) = InterpolationHelper.ComputeParametersSurface(dataPoints, ParameterizationType.Chord);
            double maxError = 0.0;
            double sumSquaredError = 0.0;
            int count = 0;

            for (int u = 0; u < dataU; u++)
            {
                for (int v = 0; v < dataV; v++)
                {
                    var evaluated = surface.GetPos(uk[u], vl[v]);
                    double error = evaluated.DistanceTo(dataPoints[u][v]);
                    maxError = Math.Max(maxError, error);
                    sumSquaredError += error * error;
                    count++;
                }
            }

            double rmsError = Math.Sqrt(sumSquaredError / count);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(maxError, Is.LessThan(0.3), "Max error should be reasonable");
                Assert.That(rmsError, Is.LessThan(0.1), "RMS error should be small");
            }
        }

        [Test]
        public void ApproximateCurve_InvalidInputs_ThrowsException()
        {
            // Test: Not enough points
            Assert.Throws<ArgumentException>(() =>
                LeastSquaresApproximator.ApproximateCurve(
                    new Vector3Double[] { new Vector3Double(0, 0, 0) }, 2, 3));

            // Test: Too many control points
            var points = new Vector3Double[5];
            for (int i = 0; i < 5; i++)
                points[i] = new Vector3Double(i, 0, 0);

            Assert.Throws<ArgumentException>(() =>
                LeastSquaresApproximator.ApproximateCurve(points, 2, 6));

            // Test: Control points <= degree
            Assert.Throws<ArgumentException>(() =>
                LeastSquaresApproximator.ApproximateCurve(points, 3, 3));

            // Test: Null points
            Assert.Throws<ArgumentNullException>(() =>
                LeastSquaresApproximator.ApproximateCurve(null!, 2, 3));
        }

        [Test]
        public void ApproximateSurface_InvalidInputs_ThrowsException()
        {
            // Test: Not enough rows
            Assert.Throws<ArgumentException>(() =>
                LeastSquaresApproximator.ApproximateSurface(
                    new Vector3Double[][] { new Vector3Double[3] }, 2, 2, 3, 3));

            // Test: Too many control points
            var points = new Vector3Double[5][];
            for (int i = 0; i < 5; i++)
            {
                points[i] = new Vector3Double[4];
                for (int j = 0; j < 4; j++)
                    points[i][j] = new Vector3Double(i, j, 0);
            }

            Assert.Throws<ArgumentException>(() =>
                LeastSquaresApproximator.ApproximateSurface(points, 2, 2, 6, 3));

            Assert.Throws<ArgumentException>(() =>
                LeastSquaresApproximator.ApproximateSurface(points, 2, 2, 3, 5));

            // Test: Null points
            Assert.Throws<ArgumentNullException>(() =>
                LeastSquaresApproximator.ApproximateSurface(null!, 2, 2, 3, 3));
        }
    }
}
