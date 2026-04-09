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
            knots = [0, 0.2, 0.5, 1, 2, 2, 3, 3.5, 4];
            degree = 3;
            span = evaluator.ExposeFindSpan(degree, knots, 0.1);
            Assert.That(span, Is.EqualTo(3));
        }

        /// <summary>
        /// B-spline basis functions sum to 1 (partition of unity) at any parameter.
        /// </summary>
        [Test]
        public void BasisFunction_PartitionOfUnity()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 0, 0, 1, 2, 3, 3, 3, 3 };
            int degree = 3;
            int n = knots.Length - degree - 2; // last control point index

            double[] sampleParams = { 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0 };

            foreach (double u in sampleParams)
            {
                double sum = 0.0;
                for (int i = 0; i <= n; i++)
                    sum += evaluator.ExposeBasisFunction(i, degree, knots, u);
                Assert.That(sum, Is.EqualTo(1.0).Within(1e-12),
                    $"Partition of unity violated at u={u}: sum={sum}");
            }
        }

        /// <summary>
        /// B-spline basis functions are non-negative everywhere.
        /// </summary>
        [Test]
        public void BasisFunction_NonNegativity()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 0, 0, 1, 2, 3, 3, 3, 3 };
            int degree = 3;
            int n = knots.Length - degree - 2;

            for (int sample = 0; sample <= 30; sample++)
            {
                double u = 3.0 * sample / 30.0;
                for (int i = 0; i <= n; i++)
                {
                    double val = evaluator.ExposeBasisFunction(i, degree, knots, u);
                    Assert.That(val, Is.GreaterThanOrEqualTo(-1e-14),
                        $"Negative basis function N_{i},{degree} at u={u}: {val}");
                }
            }
        }

        /// <summary>
        /// Linear basis functions (degree 1) on uniform knots produce hat functions.
        /// </summary>
        [Test]
        public void BasisFunction_Degree1_HatFunction()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            // Degree 1, 3 control points -> knots [0,0,0.5,1,1]
            double[] knots = new double[] { 0, 0, 0.5, 1, 1 };
            int degree = 1;

            // At u=0: N_0=1, N_1=0, N_2=0
            Assert.That(evaluator.ExposeBasisFunction(0, degree, knots, 0.0), Is.EqualTo(1.0).Within(1e-12));
            Assert.That(evaluator.ExposeBasisFunction(1, degree, knots, 0.0), Is.EqualTo(0.0).Within(1e-12));

            // At u=0.5: N_0=0, N_1=1, N_2=0 (N_1 peaks at the interior knot)
            double n0 = evaluator.ExposeBasisFunction(0, degree, knots, 0.5);
            double n1 = evaluator.ExposeBasisFunction(1, degree, knots, 0.5);
            Assert.That(n0, Is.EqualTo(0.0).Within(1e-12));
            Assert.That(n1, Is.EqualTo(1.0).Within(1e-12)); // at knot, N_1=1

            // At u=1: N_last=1
            Assert.That(evaluator.ExposeBasisFunction(2, degree, knots, 1.0), Is.EqualTo(1.0).Within(1e-12));

            // Intermediate values verify piecewise-linear (hat) shape:
            // N_0 linearly ramps from 1 at u=0 to 0 at u=0.5 -> at u=0.25, N_0=0.5
            Assert.That(evaluator.ExposeBasisFunction(0, degree, knots, 0.25), Is.EqualTo(0.5).Within(1e-12));
            // N_1 ramps up from 0 at u=0 to 1 at u=0.5, then back to 0 at u=1.0
            //   at u=0.25 (half way on [0,0.5]), N_1=0.5
            Assert.That(evaluator.ExposeBasisFunction(1, degree, knots, 0.25), Is.EqualTo(0.5).Within(1e-12));
            //   at u=0.75 (half way on [0.5,1.0]), N_1=0.5
            Assert.That(evaluator.ExposeBasisFunction(1, degree, knots, 0.75), Is.EqualTo(0.5).Within(1e-12));
            // N_2 ramps from 0 at u=0.5 to 1 at u=1.0 -> at u=0.75, N_2=0.5
            Assert.That(evaluator.ExposeBasisFunction(2, degree, knots, 0.75), Is.EqualTo(0.5).Within(1e-12));
        }

        /// <summary>
        /// First derivative of B-spline basis sums to zero (derivative of partition of unity = 0).
        /// </summary>
        [Test]
        public void BasisFunctionDerivative_SumsToZero()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 0, 0, 1, 2, 3, 3, 3, 3 };
            int degree = 3;
            int n = knots.Length - degree - 2;

            // Sample inside the domain, avoiding knot values for derivative continuity
            double[] sampleParams = { 0.5, 1.5, 2.5 };

            foreach (double u in sampleParams)
            {
                double sum = 0.0;
                for (int i = 0; i <= n; i++)
                    sum += evaluator.ExposeBasisFunctionDerivative(i, degree, knots, u, 1);
                Assert.That(sum, Is.EqualTo(0.0).Within(1e-10),
                    $"Sum of first derivatives should be 0 at u={u}: got {sum}");
            }
        }

        /// <summary>
        /// DeBoor algorithm on a degree-1 curve with two control points reproduces linear interpolation.
        /// </summary>
        [Test]
        public void DeBoor_Degree1_LinearInterpolation()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 1, 1 };
            int degree = 1;

            Vector4Double[] cps = new Vector4Double[]
            {
                new Vector4Double(0, 0, 0, 1),
                new Vector4Double(10, 5, 0, 1)
            };

            int span0 = evaluator.ExposeFindSpan(degree, knots, 0.0);
            int span1 = evaluator.ExposeFindSpan(degree, knots, 1.0);
            int spanMid = evaluator.ExposeFindSpan(degree, knots, 0.5);

            var p0 = evaluator.ExposeDeBoor(degree, knots, span0, cps, 0.0);
            var p1 = evaluator.ExposeDeBoor(degree, knots, span1, cps, 1.0);
            var pm = evaluator.ExposeDeBoor(degree, knots, spanMid, cps, 0.5);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(p0.X / p0.W, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(p0.Y / p0.W, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(p1.X / p1.W, Is.EqualTo(10.0).Within(1e-12));
                Assert.That(p1.Y / p1.W, Is.EqualTo(5.0).Within(1e-12));
                Assert.That(pm.X / pm.W, Is.EqualTo(5.0).Within(1e-12));
                Assert.That(pm.Y / pm.W, Is.EqualTo(2.5).Within(1e-12));
            }
        }

        /// <summary>
        /// DeBoor on a degree-2 (quadratic) curve passes through end control points.
        /// </summary>
        [Test]
        public void DeBoor_Degree2_Quadratic_EndPointInterpolation()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 0, 1, 1, 1 };
            int degree = 2;

            Vector4Double[] cps = new Vector4Double[]
            {
                new Vector4Double(0, 0, 0, 1),
                new Vector4Double(1, 2, 0, 1),
                new Vector4Double(2, 0, 0, 1)
            };

            int span0 = evaluator.ExposeFindSpan(degree, knots, 0.0);
            int span1 = evaluator.ExposeFindSpan(degree, knots, 1.0);

            var p0 = evaluator.ExposeDeBoor(degree, knots, span0, cps, 0.0);
            var p1 = evaluator.ExposeDeBoor(degree, knots, span1, cps, 1.0);

            using (Assert.EnterMultipleScope())
            {
                // Clamped B-spline interpolates first and last control points
                Assert.That(p0.X / p0.W, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(p0.Y / p0.W, Is.EqualTo(0.0).Within(1e-12));
                Assert.That(p1.X / p1.W, Is.EqualTo(2.0).Within(1e-12));
                Assert.That(p1.Y / p1.W, Is.EqualTo(0.0).Within(1e-12));
            }
        }

        /// <summary>
        /// FindSpan returns consistent results for out-of-range parameters (clamping behavior).
        /// </summary>
        [Test]
        public void FindSpan_ClampingBehavior()
        {
            DummyEvaluator evaluator = new DummyEvaluator();
            double[] knots = new double[] { 0, 0, 0, 1, 1, 1 };
            int degree = 2;

            // Below minimum: should return degree
            Assert.That(evaluator.ExposeFindSpan(degree, knots, -5.0), Is.EqualTo(degree));
            // Above maximum: should return n = knots.Length - degree - 2
            int expected = knots.Length - degree - 2;
            Assert.That(evaluator.ExposeFindSpan(degree, knots, 5.0), Is.EqualTo(expected));
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
