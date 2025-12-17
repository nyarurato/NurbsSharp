using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Analysis;
using System;
using System.Linq;

namespace UnitTests.Analysis
{
    [TestFixture]
    public class CurveArcLengthSamplingTest
    {
        [Test]
        public void GetPointsAtUniformArcLength_Line_ProducesEquallySpacedPoints()
        {
            // Straight line from (0,0,0) to (10,0,0)
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, 20);
            var points = param.GetPointsAtUniformArcLength(11, includeEndpoints: true);

            Assert.That(points.Length, Is.EqualTo(11));

            // Check endpoints
            Assert.That(points[0].X, Is.EqualTo(0.0).Within(1e-10));
            Assert.That(points[10].X, Is.EqualTo(10.0).Within(1e-10));

            // Check uniform spacing (should be exactly 1.0 unit apart for straight line)
            for (int i = 0; i < points.Length - 1; i++)
            {
                double dist = (points[i + 1] - points[i]).magnitude;
                Assert.That(dist, Is.EqualTo(1.0).Within(1e-9), $"Distance between point {i} and {i+1} should be 1.0");
            }
        }

        [Test]
        public void GetPointsAtUniformArcLength_Circle_ProducesUniformChordLengths()
        {
            // Circle R=5
            int degree = 2;
            double[] knots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            double w = 0.70710678;
            ControlPoint[] controlPoints = [
                new ControlPoint(5 ,  0, 0, 1),
                new ControlPoint(5 ,  5, 0, w),
                new ControlPoint(0 ,  5, 0, 1),
                new ControlPoint(-5,  5, 0, w),
                new ControlPoint(-5,  0, 0, 1),
                new ControlPoint(-5, -5, 0, w),
                new ControlPoint(0 , -5, 0, 1),
                new ControlPoint(5 , -5, 0, w),
                new ControlPoint(5 ,  0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, 80);
            var points = param.GetPointsAtUniformArcLength(50, includeEndpoints: true);

            Assert.That(points.Length, Is.EqualTo(50));

            // All points should be approximately on the circle (radius 5)
            foreach (var pt in points)
            {
                double radius = Math.Sqrt(pt.X * pt.X + pt.Y * pt.Y);
                Assert.That(radius, Is.EqualTo(5.0).Within(0.02), "Point should lie on circle radius 5");
            }

            // Check that arc-length spacing is uniform (chord lengths should be similar)
            double firstChord = (points[1] - points[0]).magnitude;
            for (int i = 1; i < points.Length - 1; i++)
            {
                double chord = (points[i + 1] - points[i]).magnitude;
                // Allow some tolerance due to chord vs arc length difference
                Assert.That(chord, Is.EqualTo(firstChord).Within(0.05), 
                    $"Chord {i} should be similar to first chord (uniform arc-length)");
            }
        }

        [Test]
        public void GetParametersAtUniformArcLength_Line_ProducesLinearProgression()
        {
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, 20);
            var parameters = param.GetParametersAtUniformArcLength(11, includeEndpoints: true);

            Assert.That(parameters.Length, Is.EqualTo(11));
            Assert.That(parameters[0], Is.EqualTo(0.0).Within(1e-10));
            Assert.That(parameters[10], Is.EqualTo(1.0).Within(1e-10));

            // For straight line, arc-length parameterization = linear parameterization
            for (int i = 0; i < parameters.Length; i++)
            {
                double expected = i / 10.0;
                Assert.That(parameters[i], Is.EqualTo(expected).Within(1e-8));
            }
        }

        [Test]
        public void GetPointsAtUniformArcLength_ExcludeEndpoints_DoesNotIncludeExactEnds()
        {
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, 20);
            var points = param.GetPointsAtUniformArcLength(10, includeEndpoints: false);

            Assert.That(points.Length, Is.EqualTo(10));

            // First point should NOT be at x=0
            Assert.That(points[0].X, Is.GreaterThan(0.0));
            Assert.That(points[0].X, Is.EqualTo(10.0 / 11.0).Within(1e-8));

            // Last point should NOT be at x=10
            Assert.That(points[9].X, Is.LessThan(10.0));
            Assert.That(points[9].X, Is.EqualTo(10.0 * 10.0 / 11.0).Within(1e-8));
        }

        [Test]
        public void GetPointsAtUniformArcLength_MinimumCount_ThrowsForInvalidCount()
        {
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, 10);

            Assert.Throws<ArgumentOutOfRangeException>(() => 
                param.GetPointsAtUniformArcLength(1));
            
            Assert.Throws<ArgumentOutOfRangeException>(() => 
                param.GetPointsAtUniformArcLength(0));
        }

        [Test]
        public void GetPointsAtUniformArcLength_HighCurvatureCurve_MaintainsUniformSpacing()
        {
            // Parabola-like curve with varying curvature
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(5, 10, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2);
            var curve = new NurbsCurve(2, knotVector, controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, 50);
            var points = param.GetPointsAtUniformArcLength(20, includeEndpoints: true);

            Assert.That(points.Length, Is.EqualTo(20));

            // Compute arc-lengths between consecutive points using the parameterization
            double expectedStep = param.TotalLength / 19.0;
            
            for (int i = 0; i < points.Length - 1; i++)
            {
                // Get parameters for these points
                double u0 = param.GetParameterAtArcLength(expectedStep * i);
                double u1 = param.GetParameterAtArcLength(expectedStep * (i + 1));
                
                // Arc length between them should be approximately expectedStep
                double s0 = param.GetArcLengthAt(u0);
                double s1 = param.GetArcLengthAt(u1);
                double arcSpacing = s1 - s0;
                
                Assert.That(arcSpacing, Is.EqualTo(expectedStep).Within(0.02), 
                    $"Arc-length spacing at segment {i} should be uniform");
            }
        }
    }
}
