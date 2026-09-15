using System;
using System.IO;
using NUnit.Framework;
using NurbsSharp.Core;

namespace UnitTests.TestInfrastructure
{
    [TestFixture]
    internal sealed class TestInfrastructureTests
    {
        [Test]
        public void ScalarComparison_IsSymmetricAndIncludesBoundary()
        {
            var tolerance = new NumericTolerance("self-test", 1.0, 0.0);

            NumericAssert.Scalar(0.0, 1.0, tolerance);
            NumericAssert.Scalar(1.0, 0.0, tolerance);
        }

        [Test]
        public void ScalarComparison_FailsForNonFiniteAndReportsScale()
        {
            AssertionException nonFinite = Assert.Throws<AssertionException>(() =>
                NumericAssert.Scalar(0.0, double.NaN, TestTolerances.AnalyticScalar, 42.0, "nan-case"))!;
            AssertionException outside = Assert.Throws<AssertionException>(() =>
                NumericAssert.Scalar(0.0, 2.0, new NumericTolerance("self-test", 1.0, 0.0), 42.0, "outside"))!;

            Assert.That(nonFinite.Message, Does.Contain("finite"));
            Assert.That(outside.Message, Does.Contain("allowed=1"));
            Assert.That(outside.Message, Does.Contain("scale=42"));
        }

        [Test]
        public void VectorComparison_UsesNormRatherThanIndependentComponents()
        {
            var tolerance = new NumericTolerance("self-test", 1.0, 0.0);

            NumericAssert.Vector(Vector3Double.Zero, new Vector3Double(0.6, 0.6, 0.0), tolerance, 1.0);
            Assert.Throws<AssertionException>(() =>
                NumericAssert.Vector(Vector3Double.Zero, new Vector3Double(0.8, 0.8, 0.0), tolerance, 1.0));
        }

        [Test]
        public void DomainSamples_MapArbitraryDomainAndExposeKnotSides()
        {
            var domain = new ParameterDomain(2.0, 5.0);
            var samples = DomainSamples.EndpointsAndInterior(domain);
            var sides = DomainSamples.AroundInteriorKnot(3.5);

            Assert.That(samples, Is.EqualTo(new[] { 2.0, 2.75, 3.5, 4.25, 5.0 }));
            Assert.That(sides[0], Is.LessThan(3.5));
            Assert.That(sides[1], Is.EqualTo(3.5));
            Assert.That(sides[2], Is.GreaterThan(3.5));
        }

        [Test]
        public void TemporaryDirectory_IsUniqueAndRemovedOnDispose()
        {
            string path;
            using (var directory = new TemporaryDirectory())
            {
                path = directory.Path;
                Assert.That(Directory.Exists(path), Is.True);
            }

            Assert.That(Directory.Exists(path), Is.False);
        }
    }
}
