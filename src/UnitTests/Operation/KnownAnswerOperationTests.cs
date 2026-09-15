using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Operation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Operation
{
    [TestFixture]
    internal sealed class KnownAnswerOperationTests
    {
        [Test]
        public void InsertKnot_QuadraticBezier_HasDeCasteljauStructure()
        {
            var original = NurbsFixtures.QuadraticBezier();
            var inserted = KnotOperator.InsertKnot(original, 0.5, 1);

            Assert.That(inserted.Degree, Is.EqualTo(2));
            Assert.That(inserted.KnotVector.Knots, Is.EqualTo(new[] { 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0 }));
            AssertControlPoints(
                inserted.ControlPoints,
                [
                    new Vector3Double(0.0, 0.0, 0.0),
                    new Vector3Double(0.5, 1.0, 0.0),
                    new Vector3Double(2.0, 1.0, 0.0),
                    new Vector3Double(3.0, 0.0, 0.0),
                ],
                "inserted");

            foreach (double u in DomainSamples.EndpointsAndInterior(new ParameterDomain(0.0, 1.0)))
            {
                Vector3Double expected = QuadraticBezierClosedForm(u);
                NumericAssert.Vector(expected, CurveEvaluator.Evaluate(inserted, u), TestTolerances.AnalyticPosition, 3.0, $"inserted C({u})");
            }
        }

        [Test]
        public void SplitCurve_QuadraticBezier_HasDeCasteljauChildren()
        {
            var (left, right) = SplitOperator.SplitCurve(NurbsFixtures.QuadraticBezier(), 0.5);

            Assert.That(left.KnotVector.Knots, Is.EqualTo(new[] { 0.0, 0.0, 0.0, 0.5, 0.5, 0.5 }));
            Assert.That(right.KnotVector.Knots, Is.EqualTo(new[] { 0.5, 0.5, 0.5, 1.0, 1.0, 1.0 }));
            AssertControlPoints(left.ControlPoints, [new(0.0, 0.0, 0.0), new(0.5, 1.0, 0.0), new(1.25, 1.0, 0.0)], "left");
            AssertControlPoints(right.ControlPoints, [new(1.25, 1.0, 0.0), new(2.0, 1.0, 0.0), new(3.0, 0.0, 0.0)], "right");

            NumericAssert.Vector(new Vector3Double(1.25, 1.0, 0.0), CurveEvaluator.Evaluate(left, 0.5), TestTolerances.AnalyticPosition, 3.0, "left seam");
            NumericAssert.Vector(new Vector3Double(1.25, 1.0, 0.0), CurveEvaluator.Evaluate(right, 0.5), TestTolerances.AnalyticPosition, 3.0, "right seam");
        }

        [Test]
        public void ElevateDegree_QuadraticBezier_HasAnalyticCubicControlPoints()
        {
            var elevated = DegreeOperator.ElevateDegree(NurbsFixtures.QuadraticBezier(), 1);

            Assert.That(elevated.Degree, Is.EqualTo(3));
            Assert.That(elevated.KnotVector.Knots, Is.EqualTo(new[] { 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 }));
            AssertControlPoints(
                elevated.ControlPoints,
                [
                    new Vector3Double(0.0, 0.0, 0.0),
                    new Vector3Double(2.0 / 3.0, 4.0 / 3.0, 0.0),
                    new Vector3Double(5.0 / 3.0, 4.0 / 3.0, 0.0),
                    new Vector3Double(3.0, 0.0, 0.0),
                ],
                "elevated");

            foreach (double u in DomainSamples.EndpointsAndInterior(new ParameterDomain(0.0, 1.0)))
                NumericAssert.Vector(QuadraticBezierClosedForm(u), CurveEvaluator.Evaluate(elevated, u), TestTolerances.AnalyticPosition, 3.0, $"elevated C({u})");
        }

        private static Vector3Double QuadraticBezierClosedForm(double u)
        {
            double oneMinusU = 1.0 - u;
            return oneMinusU * oneMinusU * new Vector3Double(0.0, 0.0, 0.0)
                + 2.0 * u * oneMinusU * new Vector3Double(1.0, 2.0, 0.0)
                + u * u * new Vector3Double(3.0, 0.0, 0.0);
        }

        private static void AssertControlPoints(ControlPoint[] actual, Vector3Double[] expected, string context)
        {
            Assert.That(actual, Has.Length.EqualTo(expected.Length));
            for (int i = 0; i < expected.Length; i++)
            {
                NumericAssert.Vector(expected[i], actual[i].Position, TestTolerances.OperationStructure, 3.0, $"{context} control point {i}");
                NumericAssert.Scalar(1.0, actual[i].Weight, TestTolerances.OperationStructure, 1.0, $"{context} weight {i}");
            }
        }
    }
}
