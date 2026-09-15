using System;
using NUnit.Framework;
using NurbsSharp.Analysis;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Evaluation
{
    [TestFixture]
    internal sealed class KnownAnswerSurfaceTests
    {
        [Test]
        public void ArbitraryDomainBilinearRectangle_MatchesAffineClosedForm()
        {
            var surface = NurbsFixtures.ArbitraryDomainRectangle();

            NumericAssert.Vector(new Vector3Double(1.0, 2.25, 0.0), SurfaceEvaluator.Evaluate(surface, 2.75, 13.0), TestTolerances.AnalyticPosition, 5.0, "S(2.75,13)");
            NumericAssert.Vector(new Vector3Double(0.0, 0.0, 0.0), SurfaceEvaluator.Evaluate(surface, 2.0, 10.0), TestTolerances.AnalyticPosition, 5.0, "S minimum corner");
            NumericAssert.Vector(new Vector3Double(4.0, 3.0, 0.0), SurfaceEvaluator.Evaluate(surface, 5.0, 14.0), TestTolerances.AnalyticPosition, 5.0, "S maximum corner");

            var first = SurfaceEvaluator.EvaluateFirstDerivative(surface, 2.75, 13.0);
            NumericAssert.Vector(new Vector3Double(4.0 / 3.0, 0.0, 0.0), first.u_deriv, TestTolerances.AnalyticFirstDerivative, 4.0 / 3.0, "S_u");
            NumericAssert.Vector(new Vector3Double(0.0, 3.0 / 4.0, 0.0), first.v_deriv, TestTolerances.AnalyticFirstDerivative, 3.0 / 4.0, "S_v");

            var second = SurfaceEvaluator.EvaluateSecondDerivative(surface, 2.75, 13.0);
            NumericAssert.Vector(Vector3Double.Zero, second.uu_deriv, TestTolerances.AnalyticSecondDerivative, 1.0, "S_uu");
            NumericAssert.Vector(Vector3Double.Zero, second.uv_deriv, TestTolerances.AnalyticSecondDerivative, 1.0, "S_uv");
            NumericAssert.Vector(Vector3Double.Zero, second.vv_deriv, TestTolerances.AnalyticSecondDerivative, 1.0, "S_vv");
            NumericAssert.Vector(new Vector3Double(0.0, 0.0, 1.0), SurfaceAnalyzer.EvaluateNormal(surface, 2.75, 13.0), TestTolerances.AnalyticPosition, 1.0, "surface normal");
            // PB-005: SurfaceAnalyzer.SurfaceArea currently probes v=0 while checking
            // degeneracy, so the analytic area=12 case throws on this [10,14] V domain.
            // Keep that correct expectation for the dedicated bug-fix PR instead of
            // weakening it or approving the exception as characterization here.
        }

        [Test]
        public void RationalQuarterCylinder_MatchesAnalyticMidpointAndOrthogonalPartials()
        {
            var surface = NurbsFixtures.QuarterCylinder();
            double q = Math.Sqrt(2.0) / 2.0;

            Vector3Double point = SurfaceEvaluator.Evaluate(surface, 0.5, 0.5);
            var first = SurfaceEvaluator.EvaluateFirstDerivative(surface, 0.5, 0.5);

            NumericAssert.Vector(new Vector3Double(q, q, 1.0), point, TestTolerances.AnalyticPosition, 2.0, "quarter-cylinder midpoint");
            NumericAssert.Scalar(1.0, point.X * point.X + point.Y * point.Y, TestTolerances.AnalyticScalar, 1.0, "quarter-cylinder radius squared");
            NumericAssert.Vector(new Vector3Double(0.0, 0.0, 2.0), first.u_deriv, TestTolerances.AnalyticFirstDerivative, 2.0, "quarter-cylinder S_u");
            NumericAssert.Scalar(0.0, Vector3Double.Dot(first.u_deriv, first.v_deriv), TestTolerances.AnalyticScalar, first.u_deriv.magnitude * first.v_deriv.magnitude, "S_u dot S_v");
        }

        [Test]
        public void ConstantRationalSurface_HasConstantPositionAndZeroDefinedPartials()
        {
            var surface = NurbsFixtures.ConstantSurface();
            var expected = new Vector3Double(7.0, -2.0, 0.5);

            foreach (double u in new[] { 0.0, 0.3, 1.0 })
            {
                foreach (double v in new[] { 0.0, 0.7, 1.0 })
                    NumericAssert.Vector(expected, SurfaceEvaluator.Evaluate(surface, u, v), TestTolerances.AnalyticPosition, 8.0, $"constant S({u},{v})");
            }

            var first = SurfaceEvaluator.EvaluateFirstDerivative(surface, 0.3, 0.7);
            NumericAssert.Vector(Vector3Double.Zero, first.u_deriv, TestTolerances.AnalyticFirstDerivative, 8.0, "constant S_u");
            NumericAssert.Vector(Vector3Double.Zero, first.v_deriv, TestTolerances.AnalyticFirstDerivative, 8.0, "constant S_v");
            NumericAssert.Scalar(0.0, SurfaceAnalyzer.SurfaceArea(surface, 0.0, 1.0, 0.0, 1.0), TestTolerances.AnalyticArea, 1.0, "constant surface area");
        }
    }
}
