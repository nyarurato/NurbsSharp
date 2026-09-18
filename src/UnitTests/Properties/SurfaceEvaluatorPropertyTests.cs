using System;
using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using UnitTests.TestInfrastructure;

namespace UnitTests.Properties
{
    [TestFixture]
    internal sealed class SurfaceEvaluatorPropertyTests
    {
        [TestCase(2, 2)]
        [TestCase(3, 2)]
        [TestCase(4, 3)]
        [TestCase(32, 2)]
        [TestCase(2, 32)]
        public void RationalBezierSurface_IsAffineCovariantAndWeightScaleInvariant(int degreeU, int degreeV)
        {
            NurbsSurface source = CreateBezierSurface(degreeU, degreeV, 1.0, Identity);
            NurbsSurface weightScaled = CreateBezierSurface(degreeU, degreeV, 6.5, Identity);
            NurbsSurface transformed = CreateBezierSurface(degreeU, degreeV, 1.0, ApplyAffineTransform);

            foreach (double normalizedU in new[] { 0.0, 0.23, 0.5, 1.0 })
            {
                double u = -2.0 + 5.0 * normalizedU;
                foreach (double normalizedV in new[] { 0.0, 0.31, 0.77, 1.0 })
                {
                    double v = 10.0 + 4.0 * normalizedV;
                    Vector3Double sourcePoint = SurfaceEvaluator.Evaluate(source, u, v);

                    AssertFinite(sourcePoint, $"degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
                    NumericAssert.Vector(
                        sourcePoint,
                        SurfaceEvaluator.Evaluate(weightScaled, u, v),
                        TestTolerances.AnalyticPosition,
                        30.0,
                        $"global weight scale degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
                    NumericAssert.Vector(
                        ApplyAffineTransform(sourcePoint),
                        SurfaceEvaluator.Evaluate(transformed, u, v),
                        TestTolerances.AnalyticPosition,
                        70.0,
                        $"affine covariance degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
                }
            }

            NumericAssert.Vector(source.ControlPoints[0][0].Position, SurfaceEvaluator.Evaluate(source, -2.0, 10.0), TestTolerances.AnalyticPosition, 30.0, "minimum corner");
            NumericAssert.Vector(source.ControlPoints[^1][^1].Position, SurfaceEvaluator.Evaluate(source, 3.0, 14.0), TestTolerances.AnalyticPosition, 30.0, "maximum corner");
        }

        [TestCase(2, 2)]
        [TestCase(3, 2)]
        [TestCase(4, 3)]
        public void RationalBezierFirstDerivative_IsAffineCovariantAndWeightScaleInvariant(int degreeU, int degreeV)
        {
            NurbsSurface source = CreateBezierSurface(degreeU, degreeV, 1.0, Identity);
            NurbsSurface weightScaled = CreateBezierSurface(degreeU, degreeV, 6.5, Identity);
            NurbsSurface transformed = CreateBezierSurface(degreeU, degreeV, 1.0, ApplyAffineTransform);

            foreach ((double u, double v) in new[]
                     {
                         (-2.0, 10.0),
                         (-0.85, 11.24),
                         (0.5, 12.0),
                         (3.0, 14.0),
                     })
            {
                var sourceDerivative = SurfaceEvaluator.EvaluateFirstDerivative(source, u, v);
                var scaledDerivative = SurfaceEvaluator.EvaluateFirstDerivative(weightScaled, u, v);
                var transformedDerivative = SurfaceEvaluator.EvaluateFirstDerivative(transformed, u, v);

                NumericAssert.Vector(
                    sourceDerivative.u_deriv,
                    scaledDerivative.u_deriv,
                    TestTolerances.AnalyticFirstDerivative,
                    30.0,
                    $"U derivative global weight scale degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
                NumericAssert.Vector(
                    sourceDerivative.v_deriv,
                    scaledDerivative.v_deriv,
                    TestTolerances.AnalyticFirstDerivative,
                    30.0,
                    $"V derivative global weight scale degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
                NumericAssert.Vector(
                    ApplyAffineLinearTransform(sourceDerivative.u_deriv),
                    transformedDerivative.u_deriv,
                    TestTolerances.AnalyticFirstDerivative,
                    70.0,
                    $"U derivative affine covariance degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
                NumericAssert.Vector(
                    ApplyAffineLinearTransform(sourceDerivative.v_deriv),
                    transformedDerivative.v_deriv,
                    TestTolerances.AnalyticFirstDerivative,
                    70.0,
                    $"V derivative affine covariance degree=({degreeU},{degreeV}), u={u:R}, v={v:R}");
            }
        }

        [Test]
        public void AsymmetricSurface_BoundariesMatchIndependentCurveEvaluations()
        {
            NurbsSurface surface = CreateBezierSurface(3, 2, 1.0, Identity);
            var uMinimumBoundary = new NurbsCurve(surface.DegreeV, surface.KnotVectorV, surface.ControlPoints[0]);
            ControlPoint[] vMaximumColumn = new ControlPoint[surface.ControlPoints.Length];
            for (int i = 0; i < vMaximumColumn.Length; i++)
                vMaximumColumn[i] = surface.ControlPoints[i][^1];
            var vMaximumBoundary = new NurbsCurve(surface.DegreeU, surface.KnotVectorU, vMaximumColumn);

            foreach (double v in new[] { 10.0, 10.8, 12.5, Math.BitDecrement(14.0), 14.0 })
            {
                NumericAssert.Vector(
                    CurveEvaluator.Evaluate(uMinimumBoundary, v),
                    SurfaceEvaluator.Evaluate(surface, -2.0, v),
                    TestTolerances.AnalyticPosition,
                    30.0,
                    $"U-min boundary v={v:R}");
            }

            foreach (double u in new[] { -2.0, -1.0, 0.5, Math.BitDecrement(3.0), 3.0 })
            {
                NumericAssert.Vector(
                    CurveEvaluator.Evaluate(vMaximumBoundary, u),
                    SurfaceEvaluator.Evaluate(surface, u, 14.0),
                    TestTolerances.AnalyticPosition,
                    30.0,
                    $"V-max boundary u={u:R}");
            }
        }

        [Test]
        public void RationalSurface_RemainsStableAroundRepeatedKnotsAndMaximumEndpoints()
        {
            NurbsSurface source = CreateRepeatedKnotSurface(1.0);
            NurbsSurface weightScaled = CreateRepeatedKnotSurface(4.0);
            double[] uSamples = [-3.0, Math.BitDecrement(-0.5), -0.5, Math.BitIncrement(-0.5), 1.0, Math.BitDecrement(2.0), 2.0];
            double[] vSamples = [10.0, Math.BitDecrement(12.0), 12.0, Math.BitIncrement(12.0), Math.BitDecrement(14.0), 14.0];

            foreach (double u in uSamples)
            {
                foreach (double v in vSamples)
                {
                    Vector3Double sourcePoint = SurfaceEvaluator.Evaluate(source, u, v);
                    AssertFinite(sourcePoint, $"repeated knots u={u:R}, v={v:R}");
                    NumericAssert.Vector(
                        sourcePoint,
                        SurfaceEvaluator.Evaluate(weightScaled, u, v),
                        TestTolerances.AnalyticPosition,
                        30.0,
                        $"repeated-knot weight scale u={u:R}, v={v:R}");
                }
            }
        }

        private static NurbsSurface CreateBezierSurface(
            int degreeU,
            int degreeV,
            double weightScale,
            Func<Vector3Double, Vector3Double> positionTransform)
        {
            double[] knotsU = CreateBezierKnots(degreeU, -2.0, 3.0);
            double[] knotsV = CreateBezierKnots(degreeV, 10.0, 14.0);
            ControlPoint[][] controlPoints = CreateControlNet(degreeU + 1, degreeV + 1, weightScale, positionTransform);
            return new NurbsSurface(
                degreeU,
                degreeV,
                new KnotVector(knotsU, degreeU),
                new KnotVector(knotsV, degreeV),
                controlPoints);
        }

        private static NurbsSurface CreateRepeatedKnotSurface(double weightScale)
        {
            const int degreeU = 3;
            const int degreeV = 2;
            double[] knotsU = [-3.0, -3.0, -3.0, -3.0, -0.5, -0.5, 2.0, 2.0, 2.0, 2.0];
            double[] knotsV = [10.0, 10.0, 10.0, 12.0, 14.0, 14.0, 14.0];
            ControlPoint[][] controlPoints = CreateControlNet(6, 4, weightScale, Identity);
            return new NurbsSurface(
                degreeU,
                degreeV,
                new KnotVector(knotsU, degreeU),
                new KnotVector(knotsV, degreeV),
                controlPoints);
        }

        private static ControlPoint[][] CreateControlNet(
            int countU,
            int countV,
            double weightScale,
            Func<Vector3Double, Vector3Double> positionTransform)
        {
            ControlPoint[][] controlPoints = new ControlPoint[countU][];
            for (int i = 0; i < countU; i++)
            {
                controlPoints[i] = new ControlPoint[countV];
                for (int j = 0; j < countV; j++)
                {
                    var position = new Vector3Double(
                        -1.0 + 0.7 * i + 0.15 * j,
                        2.0 + 0.2 * i * i + 1.1 * j,
                        0.3 * i - 0.4 * j + 0.1 * i * j);
                    double weight = weightScale * (0.7 + 0.15 * ((2 * i + j) % 5));
                    controlPoints[i][j] = new ControlPoint(positionTransform(position), weight);
                }
            }

            return controlPoints;
        }

        private static double[] CreateBezierKnots(int degree, double start, double end)
        {
            double[] knots = new double[2 * (degree + 1)];
            for (int i = 0; i <= degree; i++)
                knots[i] = start;
            for (int i = degree + 1; i < knots.Length; i++)
                knots[i] = end;
            return knots;
        }

        private static Vector3Double Identity(Vector3Double point) => point;

        private static Vector3Double ApplyAffineTransform(Vector3Double point)
        {
            return new Vector3Double(
                2.0 * point.X + 0.25 * point.Y + 3.0,
                -0.5 * point.Y + 2.0,
                1.5 * point.Z - 0.1 * point.X - 1.0);
        }

        private static Vector3Double ApplyAffineLinearTransform(Vector3Double vector)
        {
            return new Vector3Double(
                2.0 * vector.X + 0.25 * vector.Y,
                -0.5 * vector.Y,
                1.5 * vector.Z - 0.1 * vector.X);
        }

        private static void AssertFinite(Vector3Double point, string context)
        {
            Assert.That(
                double.IsFinite(point.X) && double.IsFinite(point.Y) && double.IsFinite(point.Z),
                Is.True,
                $"Surface evaluation must be finite. point={point}, context={context}");
        }
    }
}
