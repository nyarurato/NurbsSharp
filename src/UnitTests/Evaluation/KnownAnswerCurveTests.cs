using System;
using NUnit.Framework;
using NurbsSharp.Analysis;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Evaluation
{
    [TestFixture]
    internal sealed class KnownAnswerCurveTests
    {
        [Test]
        public void ArbitraryDomainLine_MatchesAffineClosedForm()
        {
            var curve = NurbsFixtures.ArbitraryDomainLine();
            var cases = new[]
            {
                (u: 2.0, expected: new Vector3Double(1.0, 2.0, 3.0)),
                (u: 3.5, expected: new Vector3Double(3.0, 4.0, 5.0)),
                (u: 5.0, expected: new Vector3Double(5.0, 6.0, 7.0)),
            };

            foreach (var item in cases)
            {
                NumericAssert.Vector(item.expected, CurveEvaluator.Evaluate(curve, item.u), TestTolerances.AnalyticPosition, 4.0 * Math.Sqrt(3.0), $"C({item.u})");
                NumericAssert.Vector4(new Vector4Double(item.expected.X, item.expected.Y, item.expected.Z, 1.0), CurveEvaluator.EvaluateHomogeneous(curve, item.u), TestTolerances.AnalyticPosition, 7.0, $"Cw({item.u})");
            }

            var derivative = new Vector3Double(4.0 / 3.0, 4.0 / 3.0, 4.0 / 3.0);
            foreach (double u in new[] { 2.0, 3.5, Math.BitDecrement(5.0), 5.0 })
                NumericAssert.Vector(derivative, CurveEvaluator.EvaluateFirstDerivative(curve, u), TestTolerances.AnalyticFirstDerivative, derivative.magnitude, $"line derivative at u={u:R}");
            NumericAssert.Scalar(4.0 * Math.Sqrt(3.0), CurveAnalyzer.CurveLength(curve, 2.0, 5.0), TestTolerances.AnalyticScalar, 4.0 * Math.Sqrt(3.0), "line length");
        }

        [Test]
        public void QuadraticBezier_MatchesBernsteinClosedForm()
        {
            var curve = NurbsFixtures.QuadraticBezier();
            var cases = new[]
            {
                (u: 0.0, expected: new Vector3Double(0.0, 0.0, 0.0)),
                (u: 0.25, expected: new Vector3Double(0.5625, 0.75, 0.0)),
                (u: 0.5, expected: new Vector3Double(1.25, 1.0, 0.0)),
                (u: 0.75, expected: new Vector3Double(2.0625, 0.75, 0.0)),
                (u: 1.0, expected: new Vector3Double(3.0, 0.0, 0.0)),
            };

            foreach (var item in cases)
                NumericAssert.Vector(item.expected, CurveEvaluator.Evaluate(curve, item.u), TestTolerances.AnalyticPosition, 3.0, $"C({item.u})");

            // C'(u)=(2+2u, 4-8u, 0), C''(u)=(2,-8,0).
            NumericAssert.Vector(new Vector3Double(2.0, 4.0, 0.0), CurveEvaluator.EvaluateFirstDerivative(curve, 0.0), TestTolerances.AnalyticFirstDerivative, 5.0, "C'(0)");
            NumericAssert.Vector(new Vector3Double(3.0, 0.0, 0.0), CurveEvaluator.EvaluateFirstDerivative(curve, 0.5), TestTolerances.AnalyticFirstDerivative, 5.0, "C'(0.5)");
            foreach (double u in new[] { Math.BitDecrement(1.0), 1.0 })
                NumericAssert.Vector(new Vector3Double(2.0 + 2.0 * u, 4.0 - 8.0 * u, 0.0), CurveEvaluator.EvaluateFirstDerivative(curve, u), TestTolerances.AnalyticFirstDerivative, 6.0, $"C'({u:R})");
            foreach (double u in new[] { 0.0, 0.25, 0.5, 0.75, Math.BitDecrement(1.0), 1.0 })
                NumericAssert.Vector(new Vector3Double(2.0, -8.0, 0.0), CurveEvaluator.EvaluateSecondDerivative(curve, u), TestTolerances.AnalyticSecondDerivative, 9.0, $"C''({u})");
        }

        [Test]
        public void RationalQuadraticQuarterCircle_MatchesAnalyticMidpoint()
        {
            var curve = NurbsFixtures.QuarterCircle();
            double q = Math.Sqrt(2.0) / 2.0;

            NumericAssert.Vector(new Vector3Double(1.0, 0.0, 0.0), CurveEvaluator.Evaluate(curve, 0.0), TestTolerances.AnalyticPosition, 1.0, "quarter-circle start");
            NumericAssert.Vector(new Vector3Double(q, q, 0.0), CurveEvaluator.Evaluate(curve, 0.5), TestTolerances.AnalyticPosition, 1.0, "quarter-circle midpoint");
            NumericAssert.Vector(new Vector3Double(0.0, 1.0, 0.0), CurveEvaluator.Evaluate(curve, 1.0), TestTolerances.AnalyticPosition, 1.0, "quarter-circle end");
            NumericAssert.Vector(new Vector3Double(-1.17157287525381, 1.17157287525381, 0.0), CurveEvaluator.EvaluateFirstDerivative(curve, 0.5), TestTolerances.AnalyticFirstDerivative, 2.0, "quarter-circle midpoint d1");
            NumericAssert.Vector(new Vector3Double(-1.9411254969542813, -1.9411254969542813, 0.0), CurveEvaluator.EvaluateSecondDerivative(curve, 0.5), TestTolerances.AnalyticSecondDerivative, 3.0, "quarter-circle midpoint d2");
            NumericAssert.Vector(new Vector3Double(-Math.Sqrt(2.0), 0.0, 0.0), CurveEvaluator.EvaluateFirstDerivative(curve, 1.0), TestTolerances.AnalyticFirstDerivative, 2.0, "quarter-circle endpoint d1");
            NumericAssert.Vector(new Vector3Double(2.0 * Math.Sqrt(2.0) - 2.0, -2.0, 0.0), CurveEvaluator.EvaluateSecondDerivative(curve, 1.0), TestTolerances.AnalyticSecondDerivative, 3.0, "quarter-circle endpoint d2");

            foreach (double u in DomainSamples.EndpointsAndInterior(new ParameterDomain(0.0, 1.0)))
            {
                Vector3Double point = CurveEvaluator.Evaluate(curve, u);
                NumericAssert.Scalar(1.0, point.X * point.X + point.Y * point.Y, TestTolerances.AnalyticScalar, 1.0, $"radius squared at u={u}");
                Assert.That(point.X, Is.GreaterThanOrEqualTo(0.0));
                Assert.That(point.Y, Is.GreaterThanOrEqualTo(0.0));
            }
        }

        [Test]
        public void ConstantRationalCurve_RemainsConstantWithUnequalWeights()
        {
            var curve = NurbsFixtures.ConstantRationalCurve();
            var expected = new Vector3Double(7.0, -2.0, 0.5);

            foreach (double u in DomainSamples.EndpointsAndInterior(new ParameterDomain(0.0, 1.0)))
                NumericAssert.Vector(expected, CurveEvaluator.Evaluate(curve, u), TestTolerances.AnalyticPosition, 8.0, $"constant C({u})");

            foreach (double u in new[] { 0.0, 0.25, 0.5, 0.75 })
            {
                NumericAssert.Vector(Vector3Double.Zero, CurveEvaluator.EvaluateFirstDerivative(curve, u), TestTolerances.AnalyticFirstDerivative, 8.0, $"constant C'({u})");
                NumericAssert.Vector(Vector3Double.Zero, CurveEvaluator.EvaluateSecondDerivative(curve, u), TestTolerances.AnalyticSecondDerivative, 8.0, $"constant C''({u})");
            }
        }
    }
}
