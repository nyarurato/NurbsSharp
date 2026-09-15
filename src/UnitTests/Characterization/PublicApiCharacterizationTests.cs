using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using NurbsSharp.Geometry;

namespace UnitTests.Characterization
{
    [TestFixture]
    internal sealed class PublicApiCharacterizationTests
    {
        [Test]
        public void PublicApi_MatchesReviewedBaseline()
        {
            Assembly assembly = typeof(NurbsCurve).Assembly;
            string actual = PublicApiSnapshot.Generate(assembly);
            string expected = ReadBaseline();

            if (!string.Equals(expected, actual, StringComparison.Ordinal))
            {
                string diagnosticPath = Path.Combine(TestContext.CurrentContext.WorkDirectory, "PublicApiBaseline.actual.txt");
                File.WriteAllText(diagnosticPath, actual);
                Assert.Fail($"Public API differs from the reviewed baseline. Actual snapshot: {diagnosticPath}");
            }
        }

        [Test]
        public void PublicApiSnapshot_IsDeterministicAndCultureInvariant()
        {
            Assembly assembly = typeof(NurbsCurve).Assembly;
            string first = PublicApiSnapshot.Generate(assembly);
            string second = PublicApiSnapshot.Generate(assembly);
            CultureInfo originalCulture = CultureInfo.CurrentCulture;

            try
            {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");
                string underFrenchCulture = PublicApiSnapshot.Generate(assembly);
                Assert.That(underFrenchCulture, Is.EqualTo(first));
            }
            finally
            {
                CultureInfo.CurrentCulture = originalCulture;
            }

            Assert.That(second, Is.EqualTo(first));
        }

        private static string ReadBaseline()
        {
            Assembly testAssembly = typeof(PublicApiCharacterizationTests).Assembly;
            string resourceName = testAssembly.GetManifestResourceNames()
                .Single(name => name.EndsWith("Characterization.PublicApiBaseline.txt", StringComparison.Ordinal));
            using Stream stream = testAssembly.GetManifestResourceStream(resourceName)!;
            using var reader = new StreamReader(stream);
            return reader.ReadToEnd().Replace("\r\n", "\n", StringComparison.Ordinal);
        }
    }
}
