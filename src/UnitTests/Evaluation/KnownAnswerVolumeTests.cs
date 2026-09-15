using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Evaluation
{
    [TestFixture]
    internal sealed class KnownAnswerVolumeTests
    {
        [Test]
        public void IdentityTrilinearVolume_ReproducesParametersAndCorners()
        {
            var volume = NurbsFixtures.IdentityTrilinearVolume();
            var samples = new[]
            {
                new Vector3Double(0.0, 0.0, 0.0),
                new Vector3Double(1.0, 0.0, 0.0),
                new Vector3Double(0.0, 1.0, 0.0),
                new Vector3Double(0.0, 0.0, 1.0),
                new Vector3Double(1.0, 1.0, 1.0),
                new Vector3Double(0.25, 0.5, 0.75),
            };

            foreach (Vector3Double expected in samples)
            {
                var actual = VolumeEvaluator.Evaluate(volume, expected.X, expected.Y, expected.Z);
                NumericAssert.Vector(expected, new Vector3Double(actual.x, actual.y, actual.z), TestTolerances.AnalyticPosition, 1.0, $"V({expected.X},{expected.Y},{expected.Z})");
            }
        }

        [Test]
        public void IdentityTrilinearVolume_MapsArbitraryDomainsWithoutAxisTransposition()
        {
            var volume = NurbsFixtures.IdentityTrilinearVolume(2.0, 5.0, -3.0, -1.0, 10.0, 14.0);
            var actual = VolumeEvaluator.Evaluate(volume, 2.75, -2.0, 13.0);

            NumericAssert.Vector(new Vector3Double(0.25, 0.5, 0.75), new Vector3Double(actual.x, actual.y, actual.z), TestTolerances.AnalyticPosition, 1.0, "arbitrary-domain trilinear volume");
        }

        [Test]
        public void RationalTrilinearVolume_HasAnalyticCenter()
        {
            var volume = NurbsFixtures.RationalTrilinearVolume();
            var actual = VolumeEvaluator.Evaluate(volume, 0.5, 0.5, 0.5);

            NumericAssert.Vector(new Vector3Double(4.0 / 9.0, 4.0 / 9.0, 4.0 / 9.0), new Vector3Double(actual.x, actual.y, actual.z), TestTolerances.AnalyticPosition, 1.0, "rational trilinear center");
        }
    }
}
