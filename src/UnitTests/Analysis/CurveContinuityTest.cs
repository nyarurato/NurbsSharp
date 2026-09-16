using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Analysis;
using NurbsSharp.Evaluation;
using System;

namespace UnitTests.Analysis
{
    [TestFixture]
    public class CurveContinuityTest
    {
        [Test]
        public void EvaluateCurveContinuity_C0_PositionMatch()
        {
            // Two line segments meeting at (5, 0, 0) with different directions
            var curve1 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            var curve2 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(5, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 5, 0), 1)
                }
            );

            var result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C0));
            Assert.That(result.PositionGap, Is.LessThan(1e-10));
            Assert.That(result.TangentAngle, Is.GreaterThan(0.5)); // ~90 degrees
        }

        [Test]
        public void EvaluateCurveContinuity_C1_TangentMatch()
        {
            // Two line segments in same direction
            var curve1 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            var curve2 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(5, 0, 0), 1),
                    new ControlPoint(new Vector3Double(10, 0, 0), 1)
                }
            );

            var result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C2)); // Lines are C2 (zero curvature)
            Assert.That(result.PositionGap, Is.LessThan(1e-10));
            Assert.That(result.TangentAngle, Is.LessThan(1e-6));
            Assert.That(result.TangentRatio, Is.EqualTo(1.0).Within(0.01));
        }

        [Test]
        public void EvaluateCurveContinuity_G1_ParallelTangentsButDifferentMagnitudes()
        {
            // Two parabolic curves that meet at (5,0,0) with parallel tangents but different speeds
            // First curve: symmetric parabola ending horizontally at (5,0,0)
            var curve1 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(2.5, 2.5, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            // Second curve: also symmetric but with tighter control (different parameterization speed)
            // Starts at (5,0,0) and goes horizontally initially
            var curve2 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                new[] {
                    new ControlPoint(new Vector3Double(5, 0, 0), 1),
                    new ControlPoint(new Vector3Double(6.25, -1.25, 0), 1),  // Steeper curve (different speed)
                    new ControlPoint(new Vector3Double(7.5, 0, 0), 1)
                }
            );

            var result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            //Should be at least G1 (geometric tangent continuity)
            Assert.That(result.Continuity, Is.GreaterThanOrEqualTo(ContinuityType.G1));
            Assert.That(result.PositionGap, Is.LessThan(1e-10));
            Assert.That(result.TangentAngle, Is.LessThan(0.1)); // Allow reasonable tolerance for parabolas
        }

        [Test]
        public void EvaluateCurveContinuity_None_PositionGap()
        {
            var curve1 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            var curve2 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(6, 0, 0), 1),  // Gap of 1 unit
                    new ControlPoint(new Vector3Double(10, 0, 0), 1)
                }
            );

            var result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.That(result.Continuity, Is.EqualTo(ContinuityType.None));
            Assert.That(result.PositionGap, Is.EqualTo(1.0).Within(1e-6));
        }

        [Test]
        public void EvaluateCurveContinuity_Reversed_DetectsOppositeDirection()
        {
            var curve1 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            // Second curve goes backwards
            var curve2 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(5, 0, 0), 1),
                    new ControlPoint(new Vector3Double(0, 0, 0), 1)  // Opposite direction
                }
            );

            var result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(result.IsReversed, Is.True);
                Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C0));
                Assert.That(result.TangentAngle, Is.EqualTo(Math.PI).Within(1e-12));
                Assert.That(result.TangentRatio, Is.EqualTo(1.0).Within(1e-12));
            });
        }

        [Test]
        public void EvaluateCurveChainContinuity_ThreeCurves_ReturnsAllConnections()
        {
            var curve1 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            var curve2 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(5, 0, 0), 1),
                    new ControlPoint(new Vector3Double(10, 0, 0), 1)
                }
            );

            var curve3 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(10, 0, 0), 1),
                    new ControlPoint(new Vector3Double(10, 5, 0), 1)
                }
            );

            var results = CurveAnalyzer.EvaluateCurveChainContinuity(new[] { curve1, curve2, curve3 });

            Assert.That(results.Length, Is.EqualTo(2));
            
            // First connection: C2 (collinear)
            Assert.That(results[0].Continuity, Is.EqualTo(ContinuityType.C2));
            Assert.That(results[0].PositionGap, Is.LessThan(1e-10));

            // Second connection: C0 (perpendicular)
            Assert.That(results[1].Continuity, Is.EqualTo(ContinuityType.C0));
            Assert.That(results[1].PositionGap, Is.LessThan(1e-10));
        }

        [Test]
        public void EvaluateCurveContinuity_CustomTolerances_RespectsTolerance()
        {
            var curve1 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(0, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 0, 0), 1)
                }
            );

            var curve2 = new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new[] {
                    new ControlPoint(new Vector3Double(5.0001, 0, 0), 1),  // Small gap
                    new ControlPoint(new Vector3Double(10, 0, 0), 1)
                }
            );

            // With default tolerance (1e-6), should fail C0
            var result1 = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);
            Assert.That(result1.Continuity, Is.EqualTo(ContinuityType.None));

            // With relaxed tolerance (1e-3), should pass C0
            var result2 = CurveAnalyzer.EvaluateCurveContinuityAtConnection(
                curve1, curve2, positionTolerance: 1e-3);
            Assert.That(result2.Continuity, Is.GreaterThanOrEqualTo(ContinuityType.C0));
        }

        [Test]
        public void EvaluateCurveContinuity_CircleSegments_C2Continuity()
        {
            // Two 90-degree circular arc segments forming 180-degree arc
            double w = Math.Sqrt(2) / 2.0;
            
            var curve1 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                new[] {
                    new ControlPoint(new Vector3Double(5, 0, 0), 1),
                    new ControlPoint(new Vector3Double(5, 5, 0), w),
                    new ControlPoint(new Vector3Double(0, 5, 0), 1)
                }
            );

            var curve2 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                new[] {
                    new ControlPoint(new Vector3Double(0, 5, 0), 1),
                    new ControlPoint(new Vector3Double(-5, 5, 0), w),
                    new ControlPoint(new Vector3Double(-5, 0, 0), 1)
                }
            );

            var result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            // Circle segments should have at least C1 continuity
            // (Perfect C2 depends on exact parameterization matching)
            Assert.That(result.Continuity, Is.GreaterThanOrEqualTo(ContinuityType.C1));
            Assert.That(result.PositionGap, Is.LessThan(1e-6));
            Assert.That(result.TangentAngle, Is.LessThan(0.01));
        }

        [Test]
        public void EvaluateCurveContinuityAtConnection_UsesExactEndpointsOnArbitraryDomains()
        {
            // Both cubic curves have C'(connection)=(1,-3,0) and
            // C''(connection)=(0,-2,0), but their derivatives differ immediately
            // inside their respective domains. This detects endpoint offsets.
            var curve1 = new NurbsCurve(
                3,
                new KnotVector([2.0, 2.0, 2.0, 2.0, 5.0, 5.0, 5.0, 5.0], 3),
                [
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(1.0, 3.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(2.0, 3.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(3.0, 0.0, 0.0), 1.0),
                ]);

            var curve2 = new NurbsCurve(
                3,
                new KnotVector([10.0, 10.0, 10.0, 10.0, 14.0, 14.0, 14.0, 14.0], 3),
                [
                    new ControlPoint(new Vector3Double(3.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(13.0 / 3.0, -4.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(17.0 / 3.0, -40.0 / 3.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(8.0, -12.0, 0.0), 1.0),
                ]);

            CurveContinuityResult explicitEndpoints = CurveAnalyzer.EvaluateCurveContinuity(curve1, curve2, 5.0, 10.0);
            CurveContinuityResult connection = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(explicitEndpoints.Continuity, Is.EqualTo(ContinuityType.C2));
                Assert.That(connection.Continuity, Is.EqualTo(explicitEndpoints.Continuity));
                Assert.That(connection.PositionGap, Is.EqualTo(explicitEndpoints.PositionGap).Within(1e-12));
                Assert.That(connection.TangentAngle, Is.EqualTo(explicitEndpoints.TangentAngle).Within(1e-12));
                Assert.That(connection.TangentRatio, Is.EqualTo(explicitEndpoints.TangentRatio).Within(1e-12));
                Assert.That(connection.CurvatureAngle, Is.EqualTo(explicitEndpoints.CurvatureAngle).Within(1e-12));
                Assert.That(connection.CurvatureRatio, Is.EqualTo(explicitEndpoints.CurvatureRatio).Within(1e-12));
                Assert.That(connection.IsReversed, Is.EqualTo(explicitEndpoints.IsReversed));
            });
        }

        [Test]
        public void EvaluateCurveContinuity_G2_IsInvariantToParameterSpeedAndTangentialAcceleration()
        {
            // At the connection, curve1 has C'=(1,0,0), C''=(0,2,0).
            var curve1 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(-1.0, 1.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(-0.5, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                ]);

            // At the connection, curve2 has C'=(2,0,0), C''=(6,8,0).
            // Removing the tangential component and dividing by |C'|^2 gives
            // the same curvature vector (0,2,0) for both curves.
            var curve2 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(1.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(5.0, 4.0, 0.0), 1.0),
                ]);

            CurveContinuityResult result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(result.Continuity, Is.EqualTo(ContinuityType.G2));
                Assert.That(result.TangentAngle, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(result.TangentRatio, Is.EqualTo(2.0).Within(1e-12));
                Assert.That(result.CurvatureAngle, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(result.CurvatureRatio, Is.EqualTo(1.0).Within(1e-12));
            });
        }

        [Test]
        public void EvaluateCurveContinuity_C1_DoesNotImplyG2WhenCurvatureMagnitudesDiffer()
        {
            // Both curves have C'=(1,0,0), but their curvature vectors are
            // (0,2,0) and (0,4,0), respectively.
            var curve1 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(-1.0, 1.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(-0.5, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                ]);

            var curve2 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.5, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(1.0, 2.0, 0.0), 1.0),
                ]);

            CurveContinuityResult result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C1));
                Assert.That(result.TangentRatio, Is.EqualTo(1.0).Within(1e-12));
                Assert.That(result.CurvatureAngle, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(result.CurvatureRatio, Is.EqualTo(2.0).Within(1e-12));
            });
        }

        [Test]
        public void EvaluateCurveContinuity_G2_DoesNotImplyC2WhenSecondDerivativesDiffer()
        {
            // Both curves have C'=(1,0,0) and curvature vector (0,2,0).
            // curve2 has the additional tangential acceleration (3,0,0), so
            // the raw second derivatives differ and C2 does not hold.
            var curve1 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(-1.0, 1.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(-0.5, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                ]);

            var curve2 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.5, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(2.5, 1.0, 0.0), 1.0),
                ]);

            CurveContinuityResult result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(result.Continuity, Is.EqualTo(ContinuityType.G2));
                Assert.That(result.TangentRatio, Is.EqualTo(1.0).Within(1e-12));
                Assert.That(result.CurvatureAngle, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(result.CurvatureRatio, Is.EqualTo(1.0).Within(1e-12));
            });
        }

        [TestCase(-1.0, 0.01, 0.05, "positionTolerance")]
        [TestCase(double.NaN, 0.01, 0.05, "positionTolerance")]
        [TestCase(double.PositiveInfinity, 0.01, 0.05, "positionTolerance")]
        [TestCase(1e-6, -1.0, 0.05, "angleTolerance")]
        [TestCase(1e-6, double.NaN, 0.05, "angleTolerance")]
        [TestCase(1e-6, double.PositiveInfinity, 0.05, "angleTolerance")]
        [TestCase(1e-6, 3.1415926535897932 + 0.01, 0.05, "angleTolerance")]
        [TestCase(1e-6, 0.01, -1.0, "ratioTolerance")]
        [TestCase(1e-6, 0.01, double.NaN, "ratioTolerance")]
        [TestCase(1e-6, 0.01, double.PositiveInfinity, "ratioTolerance")]
        public void EvaluateCurveContinuity_InvalidTolerance_ThrowsAcrossAllEntryPoints(
            double positionTolerance,
            double angleTolerance,
            double ratioTolerance,
            string expectedParameterName)
        {
            NurbsCurve curve1 = CreateLine(-1.0, 0.0);
            NurbsCurve curve2 = CreateLine(0.0, 1.0);

            ArgumentOutOfRangeException? directException = Assert.Throws<ArgumentOutOfRangeException>(() =>
                CurveAnalyzer.EvaluateCurveContinuity(
                    curve1,
                    curve2,
                    1.0,
                    0.0,
                    positionTolerance,
                    angleTolerance,
                    ratioTolerance));
            ArgumentOutOfRangeException? connectionException = Assert.Throws<ArgumentOutOfRangeException>(() =>
                CurveAnalyzer.EvaluateCurveContinuityAtConnection(
                    curve1,
                    curve2,
                    positionTolerance,
                    angleTolerance,
                    ratioTolerance));
            ArgumentOutOfRangeException? chainException = Assert.Throws<ArgumentOutOfRangeException>(() =>
                CurveAnalyzer.EvaluateCurveChainContinuity(
                    [curve1, curve2],
                    positionTolerance,
                    angleTolerance,
                    ratioTolerance));

            Assert.Multiple(() =>
            {
                Assert.That(directException!.ParamName, Is.EqualTo(expectedParameterName));
                Assert.That(connectionException!.ParamName, Is.EqualTo(expectedParameterName));
                Assert.That(chainException!.ParamName, Is.EqualTo(expectedParameterName));
            });
        }

        [Test]
        public void EvaluateCurveContinuity_ZeroTolerances_AreAccepted()
        {
            NurbsCurve curve1 = CreateLine(-1.0, 0.0);
            NurbsCurve curve2 = CreateLine(0.0, 1.0);

            CurveContinuityResult result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(
                curve1,
                curve2,
                positionTolerance: 0.0,
                angleTolerance: 0.0,
                ratioTolerance: 0.0);

            Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C2));
        }

        [Test]
        public void EvaluateCurveContinuity_DegenerateTangent_ReturnsC0WithUndefinedMetrics()
        {
            var curve1 = new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(new Vector3Double(-1.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(0.0, 0.0, 0.0), 1.0),
                ]);
            NurbsCurve curve2 = CreateLine(0.0, 1.0);

            CurveContinuityResult result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C0));
                Assert.That(result.PositionGap, Is.EqualTo(0.0));
                Assert.That(result.TangentAngle, Is.NaN);
                Assert.That(result.TangentRatio, Is.NaN);
                Assert.That(result.CurvatureAngle, Is.NaN);
                Assert.That(result.CurvatureRatio, Is.NaN);
            });
        }

        [Test]
        public void EvaluateCurveContinuity_SmallRegularCurves_AreNotTreatedAsDegenerate()
        {
            const double scale = 1e-13;
            NurbsCurve curve1 = CreateLine(-scale, 0.0);
            NurbsCurve curve2 = CreateLine(0.0, scale);

            CurveContinuityResult result = CurveAnalyzer.EvaluateCurveContinuityAtConnection(curve1, curve2);

            Assert.Multiple(() =>
            {
                Assert.That(result.Continuity, Is.EqualTo(ContinuityType.C2));
                Assert.That(result.TangentAngle, Is.EqualTo(0.0));
                Assert.That(result.TangentRatio, Is.EqualTo(1.0));
            });
        }

        private static NurbsCurve CreateLine(double startX, double endX)
        {
            return new NurbsCurve(
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                [
                    new ControlPoint(new Vector3Double(startX, 0.0, 0.0), 1.0),
                    new ControlPoint(new Vector3Double(endX, 0.0, 0.0), 1.0),
                ]);
        }
    }
}
