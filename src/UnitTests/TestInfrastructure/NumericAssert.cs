using System;
using System.Globalization;
using NUnit.Framework;
using NurbsSharp.Core;

namespace UnitTests.TestInfrastructure
{
    internal readonly record struct NumericTolerance(string Name, double Absolute, double Relative)
    {
        public double Allowed(double actual, double expected, double referenceScale)
        {
            return Absolute + Relative * Math.Max(Math.Max(Math.Abs(actual), Math.Abs(expected)), Math.Abs(referenceScale));
        }
    }

    internal static class TestTolerances
    {
        internal static readonly NumericTolerance AnalyticScalar = new("analytic scalar", 1e-13, 1e-12);
        internal static readonly NumericTolerance AnalyticPosition = new("analytic position", 1e-13, 1e-12);
        internal static readonly NumericTolerance AnalyticFirstDerivative = new("analytic first derivative", 1e-12, 5e-11);
        internal static readonly NumericTolerance AnalyticSecondDerivative = new("analytic second derivative", 1e-11, 5e-10);
        internal static readonly NumericTolerance AnalyticArea = new("analytic area", 1e-11, 1e-10);
        internal static readonly NumericTolerance OperationStructure = new("operation structure", 1e-13, 1e-12);
    }

    internal static class NumericAssert
    {
        internal static void Scalar(
            double expected,
            double actual,
            NumericTolerance tolerance,
            double referenceScale = 0.0,
            string? context = null)
        {
            Assert.That(double.IsFinite(expected), Is.True, $"Expected value must be finite. {context}");
            Assert.That(double.IsFinite(actual), Is.True, $"Actual value must be finite. actual={Format(actual)}, {context}");

            double error = Math.Abs(actual - expected);
            double allowed = tolerance.Allowed(actual, expected, referenceScale);
            Assert.That(
                error,
                Is.LessThanOrEqualTo(allowed),
                $"{tolerance.Name} comparison failed. error={Format(error)}, allowed={Format(allowed)}, " +
                $"actual={Format(actual)}, expected={Format(expected)}, scale={Format(referenceScale)}, context={context ?? "(none)"}");
        }

        internal static void Vector(
            Vector3Double expected,
            Vector3Double actual,
            NumericTolerance tolerance,
            double characteristicLength,
            string? context = null)
        {
            AssertFinite(expected, "expected", context);
            AssertFinite(actual, "actual", context);

            double error = actual.DistanceTo(expected);
            Assert.That(double.IsFinite(characteristicLength), Is.True, "Characteristic length must be finite.");
            Assert.That(characteristicLength, Is.GreaterThanOrEqualTo(0.0), "Characteristic length must not be negative.");

            double scale = characteristicLength;
            double allowed = tolerance.Absolute + tolerance.Relative * scale;
            Assert.That(
                error,
                Is.LessThanOrEqualTo(allowed),
                $"{tolerance.Name} vector comparison failed. error={Format(error)}, allowed={Format(allowed)}, " +
                $"actual={actual}, expected={expected}, scale={Format(scale)}, context={context ?? "(none)"}");
        }

        internal static void Vector4(
            Vector4Double expected,
            Vector4Double actual,
            NumericTolerance tolerance,
            double referenceScale,
            string? context = null)
        {
            Scalar(expected.X, actual.X, tolerance, referenceScale, $"{context}.X");
            Scalar(expected.Y, actual.Y, tolerance, referenceScale, $"{context}.Y");
            Scalar(expected.Z, actual.Z, tolerance, referenceScale, $"{context}.Z");
            Scalar(expected.W, actual.W, tolerance, referenceScale, $"{context}.W");
        }

        private static void AssertFinite(Vector3Double value, string label, string? context)
        {
            Assert.That(
                double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z),
                Is.True,
                $"{label} vector must be finite. value={value}, context={context ?? "(none)"}");
        }

        private static string Format(double value)
        {
            return value.ToString("G17", CultureInfo.InvariantCulture);
        }
    }
}
