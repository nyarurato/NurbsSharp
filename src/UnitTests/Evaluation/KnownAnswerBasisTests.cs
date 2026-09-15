using System;
using NUnit.Framework;
using NurbsSharp.Evaluation;
using UnitTests.TestInfrastructure;

namespace UnitTests.Evaluation
{
    [TestFixture]
    internal sealed class KnownAnswerBasisTests
    {
        private readonly BasisProbe _basis = new();

        [Test]
        public void DegreeOneBernstein_OnArbitraryDomain_HasAnalyticValues()
        {
            double[] knots = [2.0, 2.0, 5.0, 5.0];
            var cases = new[]
            {
                (u: 2.0, n0: 1.0, n1: 0.0),
                (u: 3.5, n0: 0.5, n1: 0.5),
                (u: 5.0, n0: 0.0, n1: 1.0),
            };

            foreach (var item in cases)
            {
                NumericAssert.Scalar(item.n0, _basis.Value(0, 1, item.u, knots), TestTolerances.AnalyticScalar, 1.0, $"N0 at u={item.u}");
                NumericAssert.Scalar(item.n1, _basis.Value(1, 1, item.u, knots), TestTolerances.AnalyticScalar, 1.0, $"N1 at u={item.u}");
            }

            // N0=(5-u)/3 and N1=(u-2)/3. At u=5 these are left derivatives.
            foreach (double u in new[] { 2.0, 3.5, Math.BitDecrement(5.0), 5.0 })
            {
                NumericAssert.Scalar(-1.0 / 3.0, _basis.Derivative(0, 1, u, knots, 1), TestTolerances.AnalyticFirstDerivative, 1.0 / 3.0, $"N0' at u={u}");
                NumericAssert.Scalar(1.0 / 3.0, _basis.Derivative(1, 1, u, knots, 1), TestTolerances.AnalyticFirstDerivative, 1.0 / 3.0, $"N1' at u={u}");
            }
        }

        [Test]
        public void QuadraticBernstein_HasAnalyticBasisAndDerivatives()
        {
            double[] knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
            double[] expectedBasis = [0.5625, 0.375, 0.0625];
            double[] expectedFirst = [-1.5, 1.0, 0.5];
            double[] expectedSecond = [2.0, -4.0, 2.0];

            for (int i = 0; i < 3; i++)
            {
                NumericAssert.Scalar(expectedBasis[i], _basis.Value(i, 2, 0.25, knots), TestTolerances.AnalyticScalar, 1.0, $"N{i}");
                NumericAssert.Scalar(expectedFirst[i], _basis.Derivative(i, 2, 0.25, knots, 1), TestTolerances.AnalyticFirstDerivative, 2.0, $"N{i}'");
                NumericAssert.Scalar(expectedSecond[i], _basis.Derivative(i, 2, 0.25, knots, 2), TestTolerances.AnalyticSecondDerivative, 4.0, $"N{i}''");
            }
        }

        [Test]
        public void QuadraticBernstein_EndpointAndMidpointValuesMatchPolynomials()
        {
            double[] knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
            var cases = new[]
            {
                (u: 0.0, values: new[] { 1.0, 0.0, 0.0 }),
                (u: 0.5, values: new[] { 0.25, 0.5, 0.25 }),
                (u: 1.0, values: new[] { 0.0, 0.0, 1.0 }),
            };

            foreach (var item in cases)
            {
                for (int i = 0; i < 3; i++)
                    NumericAssert.Scalar(item.values[i], _basis.Value(i, 2, item.u, knots), TestTolerances.AnalyticScalar, 1.0, $"N{i} at u={item.u}");
            }
        }

        [Test]
        public void QuadraticBernstein_MaximumEndpointHasLeftDerivatives()
        {
            double[] knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

            foreach (double u in new[] { Math.BitDecrement(1.0), 1.0 })
            {
                double[] expectedFirst = [-2.0 * (1.0 - u), 2.0 - 4.0 * u, 2.0 * u];
                double[] expectedSecond = [2.0, -4.0, 2.0];

                for (int i = 0; i < 3; i++)
                {
                    NumericAssert.Scalar(expectedFirst[i], _basis.Derivative(i, 2, u, knots, 1), TestTolerances.AnalyticFirstDerivative, 2.0, $"N{i}' at maximum side u={u:R}");
                    NumericAssert.Scalar(expectedSecond[i], _basis.Derivative(i, 2, u, knots, 2), TestTolerances.AnalyticSecondDerivative, 4.0, $"N{i}'' at maximum side u={u:R}");
                }
            }
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
