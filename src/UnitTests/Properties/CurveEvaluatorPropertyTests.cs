using System;
using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using UnitTests.TestInfrastructure;

namespace UnitTests.Properties
{
    [TestFixture]
    internal sealed class CurveEvaluatorPropertyTests
    {
        [TestCase(2)]
        [TestCase(3)]
        [TestCase(4)]
        [TestCase(8)]
        public void RationalBezierEvaluation_IsAffineCovariantAndWeightScaleInvariant(int degree)
        {
            NurbsCurve source = CreateBezierCurve(degree, 1.0, Identity);
            NurbsCurve weightScaled = CreateBezierCurve(degree, 7.25, Identity);
            NurbsCurve transformed = CreateBezierCurve(degree, 1.0, ApplyAffineTransform);

            foreach (double normalized in new[] { 0.0, 0.17, 0.5, 0.83, 1.0 })
            {
                double u = 2.0 + 3.0 * normalized;
                Vector3Double sourcePoint = CurveEvaluator.Evaluate(source, u);

                AssertFinite(sourcePoint, $"degree={degree}, u={u:R}");
                NumericAssert.Vector(
                    sourcePoint,
                    CurveEvaluator.Evaluate(weightScaled, u),
                    TestTolerances.AnalyticPosition,
                    20.0,
                    $"global weight scale degree={degree}, u={u:R}");
                NumericAssert.Vector(
                    ApplyAffineTransform(sourcePoint),
                    CurveEvaluator.Evaluate(transformed, u),
                    TestTolerances.AnalyticPosition,
                    50.0,
                    $"affine covariance degree={degree}, u={u:R}");
            }

            NumericAssert.Vector(source.ControlPoints[0].Position, CurveEvaluator.Evaluate(source, 2.0), TestTolerances.AnalyticPosition, 20.0, $"degree={degree} start endpoint");
            NumericAssert.Vector(source.ControlPoints[^1].Position, CurveEvaluator.Evaluate(source, 5.0), TestTolerances.AnalyticPosition, 20.0, $"degree={degree} maximum endpoint");
        }

        [Test]
        public void CubicRationalCurve_RemainsStableAroundRepeatedKnotAndMaximumEndpoint()
        {
            NurbsCurve source = CreateRepeatedKnotCurve(1.0, Identity);
            NurbsCurve weightScaled = CreateRepeatedKnotCurve(3.5, Identity);
            NurbsCurve transformed = CreateRepeatedKnotCurve(1.0, ApplyAffineTransform);
            double[] samples =
            [
                -3.0,
                -2.0,
                Math.BitDecrement(-0.5),
                -0.5,
                Math.BitIncrement(-0.5),
                1.0,
                Math.BitDecrement(2.0),
                2.0,
            ];

            foreach (double u in samples)
            {
                Vector3Double sourcePoint = CurveEvaluator.Evaluate(source, u);
                AssertFinite(sourcePoint, $"repeated knot u={u:R}");
                NumericAssert.Vector(sourcePoint, CurveEvaluator.Evaluate(weightScaled, u), TestTolerances.AnalyticPosition, 20.0, $"repeated-knot weight scale u={u:R}");
                NumericAssert.Vector(ApplyAffineTransform(sourcePoint), CurveEvaluator.Evaluate(transformed, u), TestTolerances.AnalyticPosition, 50.0, $"repeated-knot affine covariance u={u:R}");
            }
        }

        private static NurbsCurve CreateBezierCurve(
            int degree,
            double weightScale,
            Func<Vector3Double, Vector3Double> positionTransform)
        {
            double[] knots = new double[2 * (degree + 1)];
            for (int i = degree + 1; i < knots.Length; i++)
                knots[i] = 5.0;
            for (int i = 0; i <= degree; i++)
                knots[i] = 2.0;

            ControlPoint[] controlPoints = new ControlPoint[degree + 1];
            for (int i = 0; i <= degree; i++)
            {
                var position = new Vector3Double(
                    -1.0 + 0.7 * i,
                    -2.0 + (i * i % 5),
                    0.1 * i + 0.5 * (i % 3));
                double weight = weightScale * (0.75 + 0.2 * (i % 4));
                controlPoints[i] = new ControlPoint(positionTransform(position), weight);
            }

            return new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
        }

        private static NurbsCurve CreateRepeatedKnotCurve(
            double weightScale,
            Func<Vector3Double, Vector3Double> positionTransform)
        {
            double[] knots = [-3.0, -3.0, -3.0, -3.0, -0.5, -0.5, 2.0, 2.0, 2.0, 2.0];
            Vector3Double[] positions =
            [
                new(-2.0, 0.0, 1.0),
                new(-1.0, 2.0, -0.5),
                new(0.0, -1.0, 1.5),
                new(1.5, 3.0, 0.25),
                new(3.0, 1.0, -1.0),
                new(4.0, 2.5, 0.5),
            ];
            double[] weights = [1.0, 0.8, 1.4, 0.65, 1.75, 1.1];
            ControlPoint[] controlPoints = new ControlPoint[positions.Length];
            for (int i = 0; i < positions.Length; i++)
                controlPoints[i] = new ControlPoint(positionTransform(positions[i]), weights[i] * weightScale);

            return new NurbsCurve(3, new KnotVector(knots, 3), controlPoints);
        }

        private static Vector3Double Identity(Vector3Double point) => point;

        private static Vector3Double ApplyAffineTransform(Vector3Double point)
        {
            return new Vector3Double(
                2.0 * point.X + 0.25 * point.Y + 3.0,
                -0.5 * point.Y + 2.0,
                1.5 * point.Z - 0.1 * point.X - 1.0);
        }

        private static void AssertFinite(Vector3Double point, string context)
        {
            Assert.That(
                double.IsFinite(point.X) && double.IsFinite(point.Y) && double.IsFinite(point.Z),
                Is.True,
                $"Curve evaluation must be finite. point={point}, context={context}");
        }
    }
}
