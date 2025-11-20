using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Generation.Interpolation;
using System;

namespace NurbsSharp.UnitTests.Generation
{
    [TestFixture]
    public class InterpolationTest
    {
        [Test]
        public void GlobalInterpolation_SimpleQuadratic()
        {
            // Arrange: 3 points for a quadratic curve
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0),
                new Vector3Double(2, 0, 0)
            };
            int degree = 2;

            // Act
            var curve = GlobalInterpolator.Interpolate(points, degree);

            // Assert
            Assert.That(curve, Is.Not.Null);
            Assert.That(curve.Degree, Is.EqualTo(degree));
            Assert.That(curve.ControlPoints.Length, Is.EqualTo(3));

            // Verify curve passes through the points
            double[] parameters = InterpolationHelper.ComputeParameters(points, ParameterizationType.Chord);
            for (int i = 0; i < points.Length; i++)
            {
                var evaluated = curve.GetPos(parameters[i]);
                Assert.That(evaluated.X, Is.EqualTo(points[i].X).Within(1e-10));
                Assert.That(evaluated.Y, Is.EqualTo(points[i].Y).Within(1e-10));
                Assert.That(evaluated.Z, Is.EqualTo(points[i].Z).Within(1e-10));
            }
        }

        [Test]
        public void GlobalInterpolation_CubicCurve()
        {
            // Arrange: 5 points for a cubic curve
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 2, 0),
                new Vector3Double(2, 3, 0),
                new Vector3Double(3, 2, 0),
                new Vector3Double(4, 0, 0)
            };
            int degree = 3;

            // Act
            var curve = GlobalInterpolator.Interpolate(points, degree);

            // Assert
            Assert.That(curve, Is.Not.Null);
            Assert.That(curve.Degree, Is.EqualTo(degree));
            Assert.That(curve.ControlPoints.Length, Is.EqualTo(5));

            // Verify curve passes through the points
            double[] parameters = InterpolationHelper.ComputeParameters(points, ParameterizationType.Chord);
            for (int i = 0; i < points.Length; i++)
            {
                var evaluated = curve.GetPos(parameters[i]);
                Assert.That(evaluated.X, Is.EqualTo(points[i].X).Within(1e-8));
                Assert.That(evaluated.Y, Is.EqualTo(points[i].Y).Within(1e-8));
                Assert.That(evaluated.Z, Is.EqualTo(points[i].Z).Within(1e-8));
            }
        }

        [Test]
        public void GlobalInterpolation_DifferentParameterizations()
        {
            // Arrange
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0),
                new Vector3Double(3, 1, 0),
                new Vector3Double(4, 0, 0)
            };
            int degree = 3;

            // Act: Test all three parameterization types
            var optionsChord = new InterpolationOptions { ParameterizationType = ParameterizationType.Chord };
            var optionsCentripetal = new InterpolationOptions { ParameterizationType = ParameterizationType.Centripetal };
            var optionsUniform = new InterpolationOptions { ParameterizationType = ParameterizationType.Uniform };

            var curveChord = GlobalInterpolator.Interpolate(points, degree, optionsChord);
            var curveCentripetal = GlobalInterpolator.Interpolate(points, degree, optionsCentripetal);
            var curveUniform = GlobalInterpolator.Interpolate(points, degree, optionsUniform);

            // Assert: All should produce valid curves
            Assert.That(curveChord, Is.Not.Null);
            Assert.That(curveCentripetal, Is.Not.Null);
            Assert.That(curveUniform, Is.Not.Null);

            // All curves should pass through endpoints
            var startPoint = points[0];
            var endPoint = points[points.Length - 1];

            var evalChordStart = curveChord.GetPos(curveChord.KnotVector.Knots[degree]);
            var evalChordEnd = curveChord.GetPos(curveChord.KnotVector.Knots[curveChord.KnotVector.Length - degree - 1]);

            Assert.That(evalChordStart.X, Is.EqualTo(startPoint.X).Within(1e-8));
            Assert.That(evalChordEnd.X, Is.EqualTo(endPoint.X).Within(1e-8));
        }

        [Test]
        public void LocalInterpolation_WithGivenKnotVector()
        {
            // Arrange
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0),
                new Vector3Double(2, 0, 0)
            };
            int degree = 2;

            // Create a simple uniform knot vector
            double[] knotVector = { 0, 0, 0, 1, 1, 1 };

            // Act
            var curve = LocalInterpolator.Interpolate(points, degree, knotVector);

            // Assert
            Assert.That(curve, Is.Not.Null);
            Assert.That(curve.Degree, Is.EqualTo(degree));
            Assert.That(curve.ControlPoints.Length, Is.EqualTo(3));
        }

        [Test]
        public void GlobalInterpolation_ThrowsOnInvalidInput()
        {
            // Test with too few points
            var tooFewPoints = new Vector3Double[] { new Vector3Double(0, 0, 0) };
            Assert.Throws<ArgumentException>(() => GlobalInterpolator.Interpolate(tooFewPoints, 2));

            // Test with degree too high
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0)
            };
            Assert.Throws<ArgumentException>(() => GlobalInterpolator.Interpolate(points, 3));

            // Test with null points
            Assert.Throws<ArgumentNullException>(() => GlobalInterpolator.Interpolate(null!, 2));
        }
    }
}
