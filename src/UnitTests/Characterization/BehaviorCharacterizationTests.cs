using System;
using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Characterization
{
    [TestFixture]
    internal sealed class BehaviorCharacterizationTests
    {
        [Test]
        public void PositionEvaluators_AcceptDomainEndpointsAndRejectOutsideParameters()
        {
            var curve = NurbsFixtures.ArbitraryDomainLine();
            var surface = NurbsFixtures.ArbitraryDomainRectangle();

            NumericAssert.Vector(new Vector3Double(1.0, 2.0, 3.0), CurveEvaluator.Evaluate(curve, 2.0), TestTolerances.AnalyticPosition, 7.0, "curve minimum endpoint");
            NumericAssert.Vector(new Vector3Double(5.0, 6.0, 7.0), CurveEvaluator.Evaluate(curve, 5.0), TestTolerances.AnalyticPosition, 7.0, "curve maximum endpoint");
            NumericAssert.Vector(new Vector3Double(0.0, 0.0, 0.0), SurfaceEvaluator.Evaluate(surface, 2.0, 10.0), TestTolerances.AnalyticPosition, 5.0, "surface minimum endpoint");
            NumericAssert.Vector(new Vector3Double(4.0, 3.0, 0.0), SurfaceEvaluator.Evaluate(surface, 5.0, 14.0), TestTolerances.AnalyticPosition, 5.0, "surface maximum endpoint");

            Assert.That(Assert.Throws<ArgumentOutOfRangeException>(() => CurveEvaluator.Evaluate(curve, Math.BitDecrement(2.0)))!.ParamName, Is.EqualTo("u"));
            Assert.That(Assert.Throws<ArgumentOutOfRangeException>(() => CurveEvaluator.Evaluate(curve, Math.BitIncrement(5.0)))!.ParamName, Is.EqualTo("u"));
            Assert.That(Assert.Throws<ArgumentOutOfRangeException>(() => SurfaceEvaluator.Evaluate(surface, Math.BitDecrement(2.0), 12.0))!.ParamName, Is.EqualTo("u"));
            Assert.That(Assert.Throws<ArgumentOutOfRangeException>(() => SurfaceEvaluator.Evaluate(surface, 3.0, Math.BitIncrement(14.0)))!.ParamName, Is.EqualTo("v"));
        }

        [Test]
        public void PositionEvaluators_NullGeometryUsesStableParameterName()
        {
            Assert.That(Assert.Throws<ArgumentNullException>(() => CurveEvaluator.Evaluate(null!, 0.0))!.ParamName, Is.EqualTo("curve"));
            Assert.That(Assert.Throws<ArgumentNullException>(() => SurfaceEvaluator.Evaluate(null!, 0.0, 0.0))!.ParamName, Is.EqualTo("surface"));
            Assert.That(Assert.Throws<ArgumentNullException>(() => VolumeEvaluator.Evaluate(null!, 0.0, 0.0, 0.0))!.ParamName, Is.EqualTo("volume"));
        }

        [Test]
        public void PublicNamespace_PreservesCurrentTesselationSpelling()
        {
            Assert.That(typeof(CurveTessellator).Namespace, Is.EqualTo("NurbsSharp.Tesselation"));
        }

        [Test]
        public void VolumeEvaluator_ReturnsNamedXyzTuple()
        {
            var result = VolumeEvaluator.Evaluate(NurbsFixtures.IdentityTrilinearVolume(), 0.25, 0.5, 0.75);

            NumericAssert.Scalar(0.25, result.x, TestTolerances.AnalyticScalar, 1.0, "tuple x");
            NumericAssert.Scalar(0.5, result.y, TestTolerances.AnalyticScalar, 1.0, "tuple y");
            NumericAssert.Scalar(0.75, result.z, TestTolerances.AnalyticScalar, 1.0, "tuple z");
        }
    }
}
