using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Evaluation;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NUnit.Framework;
using System.Numerics;

namespace UnitTests.Evaluation
{
    internal class BasisEvaluationTest
    {
        [Test]
        public void BasisFunctionTest()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 0, 0, 1, 2, 3, 3, 3, 3 };
            int degree = 3;
            int span = evaluator.ExposeFindSpan(degree, knots, 1.5);
            Assert.That(span, Is.EqualTo(4));
            span = evaluator.ExposeFindSpan(degree, knots, 0);
            Assert.That(span, Is.EqualTo(3));
            span = evaluator.ExposeFindSpan(degree, knots, 3);
            Assert.That(span, Is.EqualTo(5));
            span = evaluator.ExposeFindSpan(degree, knots, 5);
            Assert.That(span, Is.EqualTo(5));
            span = evaluator.ExposeFindSpan(degree, knots, -1);
            Assert.That(span, Is.EqualTo(3));

            // knot vector multiplicity invalid case
            knots = new double[] { 0, 0.2, 0.5, 1, 2, 2, 3, 3.5, 4 };
            degree = 3;
            span = evaluator.ExposeFindSpan(degree, knots, 0.1);
            Assert.That(span, Is.EqualTo(3));


        }
    }

    internal class DummyEvaluator : BasicEvaluator
    {
        public DummyEvaluator() { }
        public int ExposeFindSpan(int degree, double[] knots, double u)
        {
            return FindSpan(degree, knots, u);
        }
        public Vector4Double ExposeDeBoor(int p, double[] knots, int span_i, Vector4Double[] ctrlPoints, double u)
        {
            return DeBoor(p, knots, span_i, ctrlPoints, u);
        }
        public double ExposeBasisFunction(int i, int p, double[] knots, double u)
        {
            return BSplineBasisFunction(i, p, u,knots);
        }
        public double ExposeBasisFunctionDerivative(int i, int p, double[] knots, double u, int der)
        {
            return DerivativeBSplineBasisFunction(i, p, u, knots, der);
        }
    }
}
