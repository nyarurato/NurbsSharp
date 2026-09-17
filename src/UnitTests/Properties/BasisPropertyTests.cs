using System;
using System.Collections.Generic;
using NUnit.Framework;
using NurbsSharp.Evaluation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Properties
{
    [TestFixture]
    internal sealed class BasisPropertyTests
    {
        private readonly BasisProbe _basis = new();

        [TestCaseSource(nameof(BasisCases))]
        public void ValidClampedBasis_SatisfiesPartitionSupportAndDerivativeSums(BasisCase basisCase)
        {
            int basisCount = basisCase.Knots.Length - basisCase.Degree - 1;

            foreach (double u in basisCase.Samples)
            {
                double valueSum = 0.0;
                double firstDerivativeSum = 0.0;
                double secondDerivativeSum = 0.0;

                for (int i = 0; i < basisCount; i++)
                {
                    double value = _basis.Value(i, basisCase.Degree, u, basisCase.Knots);
                    double first = _basis.Derivative(i, basisCase.Degree, u, basisCase.Knots, 1);
                    double second = _basis.Derivative(i, basisCase.Degree, u, basisCase.Knots, 2);

                    Assert.That(double.IsFinite(value), Is.True, $"Basis value must be finite. case={basisCase.Name}, i={i}, u={u:R}");
                    Assert.That(value, Is.GreaterThanOrEqualTo(-1e-13), $"Basis must be non-negative. case={basisCase.Name}, i={i}, u={u:R}, value={value:R}");
                    Assert.That(double.IsFinite(first), Is.True, $"First derivative must be finite. case={basisCase.Name}, i={i}, u={u:R}");
                    Assert.That(double.IsFinite(second), Is.True, $"Second derivative must be finite. case={basisCase.Name}, i={i}, u={u:R}");

                    bool outsideSupport = u < basisCase.Knots[i]
                        || (u >= basisCase.Knots[i + basisCase.Degree + 1] && u != basisCase.DomainEnd);
                    if (outsideSupport)
                    {
                        NumericAssert.Scalar(
                            0.0,
                            value,
                            TestTolerances.AnalyticScalar,
                            1.0,
                            $"local support case={basisCase.Name}, i={i}, u={u:R}");
                    }

                    valueSum += value;
                    firstDerivativeSum += first;
                    secondDerivativeSum += second;
                }

                NumericAssert.Scalar(1.0, valueSum, TestTolerances.AnalyticScalar, 1.0, $"partition case={basisCase.Name}, u={u:R}");
                NumericAssert.Scalar(0.0, firstDerivativeSum, TestTolerances.AnalyticFirstDerivative, 1.0, $"first derivative sum case={basisCase.Name}, u={u:R}");
                NumericAssert.Scalar(0.0, secondDerivativeSum, TestTolerances.AnalyticSecondDerivative, 1.0, $"second derivative sum case={basisCase.Name}, u={u:R}");
            }
        }

        private static IEnumerable<TestCaseData> BasisCases()
        {
            yield return new TestCaseData(new BasisCase(
                "quadratic-arbitrary-domain",
                2,
                [2.0, 2.0, 2.0, 3.0, 4.0, 4.0, 4.0],
                [2.0, 2.25, Math.BitDecrement(3.0), 3.0, Math.BitIncrement(3.0), 3.75, Math.BitDecrement(4.0), 4.0]))
                .SetName("QuadraticArbitraryDomain");

            yield return new TestCaseData(new BasisCase(
                "cubic-repeated-knot",
                3,
                [-3.0, -3.0, -3.0, -3.0, -0.5, -0.5, 2.0, 2.0, 2.0, 2.0],
                [-3.0, -2.0, Math.BitDecrement(-0.5), -0.5, Math.BitIncrement(-0.5), 1.0, Math.BitDecrement(2.0), 2.0]))
                .SetName("CubicRepeatedKnot");

            yield return new TestCaseData(new BasisCase(
                "quartic-multiple-spans",
                4,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0],
                [0.0, 0.1, 0.25, 0.4, 0.5, 0.8, Math.BitDecrement(1.0), 1.0]))
                .SetName("QuarticMultipleSpans");
        }

        internal sealed record BasisCase(string Name, int Degree, double[] Knots, double[] Samples)
        {
            internal double DomainEnd => Knots[Knots.Length - Degree - 1];
        }

        private sealed class BasisProbe : BasicEvaluator
        {
            internal double Value(int i, int degree, double u, double[] knots) =>
                BSplineBasisFunction(i, degree, u, knots);

            internal double Derivative(int i, int degree, double u, double[] knots, int order) =>
                DerivativeBSplineBasisFunction(i, degree, u, knots, order);
        }
    }
}
