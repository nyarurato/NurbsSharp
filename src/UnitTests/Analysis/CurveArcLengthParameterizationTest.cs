using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Analysis;
using System;

namespace UnitTests.Analysis
{
    [TestFixture]
    public class CurveArcLengthParameterizationTest
    {
        [Test]
        public void ArcLengthParameterization_Line_IsLinearAndInvertible()
        {
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, subdivisionsPerSpan: 20);

            Assert.That(param.TotalLength, Is.EqualTo(10.0).Within(1e-10));

            double[] sampleUs = [0.0, 0.1, 0.25, 0.5, 0.9, 1.0];
            foreach (double u in sampleUs)
            {
                double s = param.GetArcLengthAt(u, accurate: true);
                Assert.That(s, Is.EqualTo(10.0 * u).Within(1e-10));

                double u2 = param.GetParameterAtArcLength(s, tolerance: 1e-12, maxIterations: 40);
                Assert.That(u2, Is.EqualTo(u).Within(1e-10));
            }
        }

        [Test]
        public void ArcLengthParameterization_Circle_RoundTripIsStable()
        {
            // Circle R=3 (full circle)
            int degree = 2;
            double[] knots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(3 ,  0, 0, 1),
                new ControlPoint(3 ,  3, 0, 0.70710678),
                new ControlPoint(0 ,  3, 0, 1),
                new ControlPoint(-3,  3, 0, 0.70710678),
                new ControlPoint(-3,  0, 0, 1),
                new ControlPoint(-3, -3, 0, 0.70710678),
                new ControlPoint(0 , -3, 0, 1),
                new ControlPoint(3 , -3, 0, 0.70710678),
                new ControlPoint(3 ,  0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var param = CurveAnalyzer.BuildArcLengthParameterization(curve, subdivisionsPerSpan: 80);

            double expected = 2.0 * Math.PI * 3.0;
            Assert.That(param.TotalLength, Is.EqualTo(expected).Within(5e-3));

            double[] sampleUs = [0.05, 0.2, 0.33, 0.6, 0.85, 0.95];
            foreach (double u in sampleUs)
            {
                double s = param.GetArcLengthAt(u, accurate: true);
                double u2 = param.GetParameterAtArcLength(s, tolerance: 1e-8, maxIterations: 40);

                using (Assert.EnterMultipleScope())
                {
                    Assert.That(u2, Is.EqualTo(u).Within(2e-4));
                    Assert.That(param.GetArcLengthAt(u2, accurate: true), Is.EqualTo(s).Within(1e-6));
                }
            }
        }
    }
}
