using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;

namespace UnitTests.Evaluation
{
    internal class CurveEvaluationTest
    {


        [Test]
        public void CubicNurbsCurve_Evaluation_DatasetA()
        {
            ControlPoint[] controlPoints1;
            // Dataset A: Cubic NURBS (non-rational)
            int degree = 3;
            double[] knots = { 0, 0, 0, 0, 0.2, 0.5, 0.7, 1, 1, 1, 1 };
            controlPoints1 = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 2.0, 0.5, 1),
                new ControlPoint(2.0, 3.0, 0.0, 1),
                new ControlPoint(4.0, 3.0, -0.5, 1),
                new ControlPoint(5.0, 1.0, 0.0, 1),
                new ControlPoint(6.0, 0.0, 0.2, 1),
                new ControlPoint(7.0, -1.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints1);

            var samples = new (double u, Vector3Double expected)[] {
                (0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (0.001, new Vector3Double(0.014955, 0.029880, 0.007448)),
                (0.100, new Vector3Double(1.133571, 1.980000, 0.315357)),
                (0.250, new Vector3Double(2.230804, 2.789583, -0.004836)),
                (0.500, new Vector3Double(3.996429, 2.550000, -0.330357)),
                (0.750, new Vector3Double(5.179977, 0.950231, -0.004919)),
                (0.900, new Vector3Double(6.123519, -0.115185, 0.103685)),
                (0.999, new Vector3Double(6.990013, -0.990013, 0.001989)),
                (1.000, new Vector3Double(7.000000, -1.000000, 0.000000))
            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);

                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
            }
        }

        [Test]
        public void QuadraticRationalNurbsCurve_Evaluation_DatasetB()
        {
            ControlPoint[] controlPoints2;
            // Dataset B: Quadratic Rational NURBS (quarter-circle approx)
            int degree = 2;
            double[] knots = { 0, 0, 0, 1, 1, 1 };
            controlPoints2 = new ControlPoint[] {
                new ControlPoint(1.0, 0.0, 0.0, 1.0),
                new ControlPoint(1.0, 1.0, 0.0, 0.707107),
                new ControlPoint(0.0, 1.0, 0.0, 1.0)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints2);

            var samples = new (double u, Vector3Double expected)[] {
                (0.000, new Vector3Double(1.000000, 0.000000, 0.0)),
                (0.001, new Vector3Double(0.999999, 0.001415, 0.0)),
                (0.250, new Vector3Double(0.929788, 0.368095, 0.0)),
                (0.500, new Vector3Double(0.707107, 0.707107, 0.0)),
                (0.750, new Vector3Double(0.368095, 0.929788, 0.0)),
                (0.900, new Vector3Double(0.144919, 0.989443, 0.0)),
                (0.999, new Vector3Double(0.001415, 0.999999, 0.0)),
                (1.000, new Vector3Double(0.000000, 1.000000, 0.0))
            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
            }
        }

        [Test]
        public void TestNormalNonUniform()
        {
            int degree = 3;
            double[] knots = { 0, 0, 0, 0, 0.2, 0.5, 0.7, 1, 1, 1, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.5, 1),
            new ControlPoint(2.0, 3.0, 0.0, 1),
            new ControlPoint(4.0, 3.0, -0.5, 1),
            new ControlPoint(5.0, 1.0, 0.0, 1),
            new ControlPoint(6.0, 0.0, 0.2, 1),
            new ControlPoint(7.0, -1.0, 0.0, 1)
        };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {
            (0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
            (0.001, new Vector3Double(0.014955, 0.029880, 0.007448)),
            (0.100, new Vector3Double(1.133571, 1.980000, 0.315357)),
            (0.250, new Vector3Double(2.230804, 2.789583, -0.004836)),
            (0.500, new Vector3Double(3.996429, 2.550000, -0.330357)),
            (0.750, new Vector3Double(5.179977, 0.950231, -0.004919)),
            (0.900, new Vector3Double(6.123519, -0.115185, 0.103685)),
            (0.999, new Vector3Double(6.990013, -0.990013, 0.001989)),
            (1.000, new Vector3Double(7.000000, -1.000000, 0.000000))
        };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
            }
        }

        [Test]
        public void TestRepeatedKnotsC0()
        {
            int degree = 3;
            double[] knots = { 0, 0, 0, 0, 0.5, 0.5, 1, 1, 1, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.0, 1),
            new ControlPoint(2.0, 2.0, 0.0, 1),
            new ControlPoint(3.0, 0.0, 0.0, 1),
            new ControlPoint(4.0, 2.0, 0.0, 1),
            new ControlPoint(5.0, 0.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {

            (0.000, new Vector3Double(0.00000000e+00, 0.00000000e+00, 0.00000000e+00)),
            (0.001, new Vector3Double(6.00000000e-03, 1.19760100e-02, 0.00000000e+00)),
            (0.100, new Vector3Double(5.96000000e-01, 9.68000000e-01, 0.00000000e+00)),
            (0.250, new Vector3Double(1.43750000e+00, 1.62500000e+00, 0.00000000e+00)),
            (0.500, new Vector3Double(2.50000000e+00, 1.00000000e+00, 0.00000000e+00)),
            (0.750, new Vector3Double(3.56250000e+00, 8.75000000e-01, 0.00000000e+00)),
            (0.900, new Vector3Double(4.40400000e+00, 7.76000000e-01, 0.00000000e+00)),
            (0.999, new Vector3Double(4.99400000e+00, 1.19520600e-02, 0.00000000e+00)),
            (1.000, new Vector3Double(5.00000000e+00, 0.00000000e+00, 0.00000000e+00))
             };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
            }
        }

        [Test]
        public void TestExtremeWeights()
        {
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.5, 1, 1, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.0, 0.1),
            new ControlPoint(2.0, 0.0, 0.0, 10),
            new ControlPoint(3.0, 1.0, 0.0, 1)
        };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {

                (0.000,new Vector3Double(0.00000000e+00, 0.00000000e+00, 0.00000000e+00)),
                (0.001,new Vector3Double(4.40977199e-04, 8.01667243e-04, 0.00000000e+00)),
                (0.100,new Vector3Double(4.96567506e-01, 7.78032037e-02, 0.00000000e+00)),
                (0.250,new Vector3Double(1.64000000e+00, 8.00000000e-02, 0.00000000e+00)),
                (0.500,new Vector3Double(1.99009901e+00, 1.98019802e-02, 0.00000000e+00)),
                (0.750,new Vector3Double(2.03646833e+00, 4.22264875e-02, 0.00000000e+00)),
                (0.900,new Vector3Double(2.15784265e+00, 1.59327066e-01, 0.00000000e+00)),
                (0.999,new Vector3Double(2.96144541e+00, 9.61445993e-01, 0.00000000e+00)),
                (1.000,new Vector3Double(3.00000000e+00, 1.00000000e+00, 0.00000000e+00))

            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
            }
        }

        [Test]
        public void TestDiscontinuousCurve()
        {
            int degree = 2;
            double[] knots = { 0, 0, 0, 1, 1, 1, 2, 2, 2 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 0.0, 0.0, 1),
            new ControlPoint(2.0, 0.0, 0.0, 1),
            new ControlPoint(3.0, 3.0, 0.0, 1),
            new ControlPoint(4.0, 3.0, 0.0, 1),
            new ControlPoint(5.0, 3.0, 0.0, 1)
        };
            var curve = new NurbsCurve(degree, new KnotVector(knots,degree), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {
                (0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (0.001, new Vector3Double(0.002000, 0.000000, 0.000000)),
                (0.100, new Vector3Double(0.200000, 0.000000, 0.000000)),
                (0.250, new Vector3Double(0.500000, 0.000000, 0.000000)),
                (0.500, new Vector3Double(1.000000, 0.000000, 0.000000)),
                (0.750, new Vector3Double(1.500000, 0.000000, 0.000000)),
                (0.900, new Vector3Double(1.800000, 0.000000, 0.000000)),
                (0.999, new Vector3Double(1.998000, 0.000000, 0.000000)),
                (1.000, new Vector3Double(3.000000, 3.000000, 0.000000))
            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
            }
        }

        [Test]
        public void NurbsCurve_Circle()
        {
            // Circle R=1
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(1 ,  0, 0, 1),
                new ControlPoint(1 ,  1, 0, 0.70710678),
                new ControlPoint(0 ,  1, 0, 1),
                new ControlPoint(-1,  1, 0, 0.70710678),
                new ControlPoint(-1,  0, 0, 1),
                new ControlPoint(-1, -1, 0, 0.70710678),
                new ControlPoint(0 , -1, 0, 1),
                new ControlPoint(1 , -1, 0, 0.70710678),
                new ControlPoint(1 ,  0, 0, 1),

            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {
                (0.000, new Vector3Double(1.000000, 0.000000, 0.000000)),
                (0.001, new Vector3Double(0.999984, 0.005663, 0.0)),
                (0.100, new Vector3Double(0.813826, 0.581108, 0.0)),
                (0.250, new Vector3Double(0.0, 1.0, 0.0)),
                (0.500, new Vector3Double(-1.0, 0.0, 0.0)),
                (0.750, new Vector3Double(0.0, -1.0, 0.0)),
                (0.900, new Vector3Double(0.8138260, -0.581108, 0.0)),
                (0.999, new Vector3Double(0.999984, -0.0056344, 0.00)),
                (1.000, new Vector3Double(1.000000, 0.000000, 0.000000))
            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.0001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.0001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.0001));
            }

        }

        [Test]
        public void CubicNurbsCurve_GetLength_DatasetA()
        {
            // Circle R=1
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(1 ,  0, 0, 1),
                new ControlPoint(1 ,  1, 0, 0.70710678),
                new ControlPoint(0 ,  1, 0, 1),
                new ControlPoint(-1,  1, 0, 0.70710678),
                new ControlPoint(-1,  0, 0, 1),
                new ControlPoint(-1, -1, 0, 0.70710678),
                new ControlPoint(0 , -1, 0, 1),
                new ControlPoint(1 , -1, 0, 0.70710678),
                new ControlPoint(1 ,  0, 0, 1),

            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            double expectedLength = Math.PI * 2;// full circle length
            double length = curve.GetLength();

            Assert.That(length, Is.EqualTo(expectedLength).Within(0.001));

            length = curve.GetLength(0.0, 0.25);
            double expectedQuarterLength = Math.PI / 2;// quarter circle length
            Assert.That(length, Is.EqualTo(expectedQuarterLength).Within(0.001));
        }

        [Test]
        public void NurbsCurve_GetLength_Line()
        {
            int degree = 1;
            double[] knots = { 0, 0, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 4.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double expectedLength = 5.0; // Length of the line from (0,0,0) to (3,4,0)
            double length = curve.GetLength();
            Assert.That(length, Is.EqualTo(expectedLength).Within(0.0001));
        }

        [Test]
        public void NurbsCurve_GetLength0()
        {
            int degree = 2;
            double[] knots = { 0, 0, 0, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(0.0, 0.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double expectedLength = 0.0; // Approximate length of the quadratic curve
            double length = curve.GetLength();
            Assert.That(length, Is.EqualTo(expectedLength).Within(0.0001));
        }

        //degree 0 curve test
        [Test]
        public void NurbsCurveDegree0()
        {
            int degree = 0;
            double[] knots = { 0, 0.1666, 0.3333, 0.5, 0.6666, 0.8333, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 0.0, 0.0, 1),
            new ControlPoint(2.0, 0.0, 0.0, 1),
            new ControlPoint(3.0, 3.0, 0.0, 1),
            new ControlPoint(4.0, 3.0, 0.0, 1),
            new ControlPoint(5.0, 3.0, 0.0, 1)
        };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {
                (0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (0.001, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (0.100, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (0.250, new Vector3Double(1.000000, 0.000000, 0.000000)),
                (0.500, new Vector3Double(3.000000, 3.000000, 0.000000)),
                (0.750, new Vector3Double(4.000000, 3.000000, 0.000000)),
                (0.900, new Vector3Double(5.000000, 3.000000, 0.000000)),
                (0.999, new Vector3Double(5.000000, 3.000000, 0.000000)),
                (1.000, new Vector3Double(5.000000, 3.000000, 0.000000))
            };

            foreach (var (u, expected) in samples)
            {
                Assert.Throws<ArgumentException>(() => CurveEvaluator.Evaluate(curve, u));
            }
        }

        //degree 0 curve test
        [Test]
        public void NurbsCurveDegree1()
        {
            int degree = 1;
            double[] knots = { 0, 0, 0.286, 0.429, 0.571, 0.714, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 0.5),
                new ControlPoint(2.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 3.0, 0.0, 0.5),
                new ControlPoint(4.0, 3.0, 0.0, 1),
                new ControlPoint(5.0, 3.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);


            var samples = new (double u, Vector3Double expected)[] {
                (0.150, new Vector3Double(0.355, 0.000000, 0.000000)),
                (0.250, new Vector3Double(0.776, 0.000000, 0.000000)),
                (0.500, new Vector3Double(2.333, 1.000000, 0.000000)),
                (0.750, new Vector3Double(4.126, 3.000000, 0.000000)),
                (0.850, new Vector3Double(4.476, 3.000000, 0.000000))
            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.001));
            }
        }


        [Test]
        public void NurbsCurveDerivationTestA()
        {
            // Circle R=1
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(1 ,  0, 0, 1),
                new ControlPoint(1 ,  1, 0, 0.70710678),
                new ControlPoint(0 ,  1, 0, 1),
                new ControlPoint(-1,  1, 0, 0.70710678),
                new ControlPoint(-1,  0, 0, 1),
                new ControlPoint(-1, -1, 0, 0.70710678),
                new ControlPoint(0 , -1, 0, 1),
                new ControlPoint(1 , -1, 0, 0.70710678),
                new ControlPoint(1 ,  0, 0, 1),

            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);


            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };
            const double h = 1e-10;
            foreach (var u in samplePoints)
            {
                var derivVal = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                //finite difference approximation
                var evalPt = CurveEvaluator.Evaluate(curve, u);
                var evalPt2 = CurveEvaluator.Evaluate(curve, u + h);
                var evalDerivPt = (evalPt2 - evalPt) / h;
                Assert.That(derivVal.X, Is.EqualTo(evalDerivPt.X).Within(0.001));
                Assert.That(derivVal.Y, Is.EqualTo(evalDerivPt.Y).Within(0.001));
                Assert.That(derivVal.Z, Is.EqualTo(evalDerivPt.Z).Within(0.001));
            }

        }

        [Test]
        public void NurbsCurveDerivationTestB()
        {
            int degree = 3;
            double[] knots = { 0, 0, 0, 0, 0.5, 0.7, 1, 1, 1, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.0, 1),
            new ControlPoint(2.0, 2.0, 0.0, 1),
            new ControlPoint(3.0, 0.0, 0.0, 1),
            new ControlPoint(4.0, 2.0, 0.0, 1),
            new ControlPoint(5.0, 0.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };

            const double h = 1e-10;
            foreach (var u in samplePoints)
            {
                var derivVal = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                //finite difference approximation
                var evalPt = CurveEvaluator.Evaluate(curve, u);
                var evalPt2 = CurveEvaluator.Evaluate(curve, u + h);
                var evalDerivPt = (evalPt2 - evalPt) / h;
                Assert.That(derivVal.X, Is.EqualTo(evalDerivPt.X).Within(0.001));
                Assert.That(derivVal.Y, Is.EqualTo(evalDerivPt.Y).Within(0.001));
                Assert.That(derivVal.Z, Is.EqualTo(evalDerivPt.Z).Within(0.001));
            }
        }

        [Test]
        public void NurbsCurveSecondDerivationTestA()
        {
            int degree = 3;
            double[] knots = { 0, 0, 0, 0, 0.5, 0.7, 1, 1, 1, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.0, 1),
            new ControlPoint(2.0, 2.0, 0.0, 1),
            new ControlPoint(3.0, 0.0, 0.0, 1),
            new ControlPoint(4.0, 2.0, 0.0, 1),
            new ControlPoint(5.0, 0.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };
            const double h = 1e-6;
            foreach (var u in samplePoints)
            {
                var derivVal = CurveEvaluator.EvaluateSecondDerivative(curve, u);
                //finite difference approximation
                var deriv1 = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                var deriv1b = CurveEvaluator.EvaluateFirstDerivative(curve, u + h);
                var evalDeriv2Pt = new Vector3Double(
                    (deriv1b.X - deriv1.X) / h,
                    (deriv1b.Y - deriv1.Y) / h,
                    (deriv1b.Z - deriv1.Z) / h
                );
                Assert.That(derivVal.X, Is.EqualTo(evalDeriv2Pt.X).Within(0.01));
                Assert.That(derivVal.Y, Is.EqualTo(evalDeriv2Pt.Y).Within(0.01));
                Assert.That(derivVal.Z, Is.EqualTo(evalDeriv2Pt.Z).Within(0.01));
            }
        }

        [Test]
        public void NurbsCurveSecondDerivationTestB()
        {
            // Circle R=2
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(2 ,  0, 0, 1),
                new ControlPoint(2 ,  2, 0, 0.70710678),
                new ControlPoint(0 ,  2, 0, 1),
                new ControlPoint(-2,  2, 0, 0.70710678),
                new ControlPoint(-2,  0, 0, 1),
                new ControlPoint(-2, -2, 0, 0.70710678),
                new ControlPoint(0 , -2, 0, 1),
                new ControlPoint(2 , -2, 0, 0.70710678),
                new ControlPoint(2 ,  0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };
            const double h = 1e-6;
            double R = 2.0;
            foreach (var u in samplePoints)
            {
                var point = CurveEvaluator.Evaluate(curve, u);
                var derivVal = CurveEvaluator.EvaluateSecondDerivative(curve, u);
                //finite difference approximation
                var deriv1 = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                var deriv1b = CurveEvaluator.EvaluateFirstDerivative(curve, u + h);
                var evalDeriv2Pt = new Vector3Double(
                    (deriv1b.X - deriv1.X) / h,
                    (deriv1b.Y - deriv1.Y) / h,
                    (deriv1b.Z - deriv1.Z) / h
                );
                Assert.That(derivVal.X, Is.EqualTo(evalDeriv2Pt.X).Within(0.01));
                Assert.That(derivVal.Y, Is.EqualTo(evalDeriv2Pt.Y).Within(0.01));
                Assert.That(derivVal.Z, Is.EqualTo(evalDeriv2Pt.Z).Within(0.01));

                // Curve curvature k = |r' x r''| / |r'|^3
                var curvature_magnitude = Vector3Double.Cross(deriv1, derivVal).magnitude / Math.Pow(deriv1.magnitude, 3);

                //Cirle is special case where second derivative can be computed exactly
                Assert.That(curvature_magnitude, Is.EqualTo(1/R).Within(0.01));
                
            }

        }

        [Test]
        public void NurbsCurveCurvatureA()
        {
            // Circle R=3
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(3 ,  0, 0, 1),
                new ControlPoint(3 ,  3, 0, 0.70710678),
                new ControlPoint(0 ,  3, 0, 1),
                new ControlPoint(-3,  3, 0, 0.70710678),
                new ControlPoint(-3,  0, 0, 1),
                new ControlPoint(-3, -3, 0, 0.70710678),
                new ControlPoint(0 , -3, 0, 1),
                new ControlPoint(3 , -3, 0, 0.70710678),
                new ControlPoint(3 ,  0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };//u=1.0 case is unstable
            double R = 3.0;
            foreach (var u in samplePoints)
            {
                var curvature = CurveEvaluator.EvaluateCurvature(curve, u);
                //Cirle is special case where curvature can be computed exactly
                Assert.That(curvature, Is.EqualTo(1 / R).Within(0.01));
            }
        }

        [Test]
        public void NurbsCurveCurvatureB()
        {
            int degree = 3;
            double[] knots = { 0, 0, 0, 0, 0.5, 0.7, 1, 1, 1, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.0, 1),
            new ControlPoint(2.0, 2.0, 0.0, 1),
            new ControlPoint(3.0, 0.0, 0.0, 1),
            new ControlPoint(4.0, 2.0, 0.0, 1),
            new ControlPoint(5.0, 0.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.999 };// u=1.0 case is unstable
            foreach (var u in samplePoints)
            {

                const double eps = 1e-8;
                
                double u0 = Math.Clamp(u, eps, 1 - eps);
                var p = CurveEvaluator.Evaluate(curve, u0);
                var d1 = CurveEvaluator.EvaluateFirstDerivative(curve, u0);
                var d2 = CurveEvaluator.EvaluateSecondDerivative(curve, u0);
                var k = CurveEvaluator.EvaluateCurvature(curve, u0);
                Console.WriteLine($"u={u} (eval at {u0}): p={p}, r'={d1}, r''={d2}, curvature={k}");
            }
        }
        [Test]
        public void FrenetFrame_Circle_OrthogonalityAndNormalization()
        {
            // Circle R=3
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(3 ,  0, 0, 1),
                new ControlPoint(3 ,  3, 0, 0.70710678),
                new ControlPoint(0 ,  3, 0, 1),
                new ControlPoint(-3,  3, 0, 0.70710678),
                new ControlPoint(-3,  0, 0, 1),
                new ControlPoint(-3, -3, 0, 0.70710678),
                new ControlPoint(0 , -3, 0, 1),
                new ControlPoint(3 , -3, 0, 0.70710678),
                new ControlPoint(3 ,  0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            double[] sampleUs = { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };//u=1.0 case is unstable
            const double tol = 1e-8;

            foreach (var u in sampleUs)
            {
                var (T, N, B) = CurveEvaluator.EvaluateFrenetFrame(curve, u);

                // If tangent is zero then frame is invalid; fail test in that case
                Assert.That(T.magnitude, Is.GreaterThan(0.0), $"Tangent is zero at u={u}");
                Assert.That(Math.Abs(T.magnitude - 1.0), Is.LessThan(tol), $"T not normalized at u={u}");

                // circle Normal is origin to point vector
                var pt = CurveEvaluator.Evaluate(curve, u);
                var expectedN = new Vector3Double(pt.X, pt.Y, pt.Z).normalized;
                Assert.That(Math.Abs(N.X) - Math.Abs(expectedN.X), Is.LessThan(tol), $"N.X incorrect at u={u}");
                Assert.That(Math.Abs(N.Y) - Math.Abs(expectedN.Y), Is.LessThan(tol), $"N.Y incorrect at u={u}");
                Assert.That(Math.Abs(N.Z) - Math.Abs(expectedN.Z), Is.LessThan(tol), $"N.Z incorrect at u={u}");

                // Binormal should be perpendicular to both and equal to cross(T,N)
                Assert.That(B.magnitude, Is.GreaterThan(0.0), $"Binormal is zero at u={u}");
                var dotTN = Math.Abs(Vector3Double.Dot(T, N));// check orthogonality
                var dotTB = Math.Abs(Vector3Double.Dot(T, B));// check orthogonality
                var dotNB = Math.Abs(Vector3Double.Dot(N, B));// check orthogonality
                Assert.That(dotTN, Is.LessThan(1e-6), $"T and N not orthogonal at u={u}");
                Assert.That(dotTB, Is.LessThan(1e-6), $"T and B not orthogonal at u={u}");
                Assert.That(dotNB, Is.LessThan(1e-6), $"N and B not orthogonal at u={u}");

                var cross = Vector3Double.Cross(T, N);
                // cross and B may differ in sign depending on orientation; compare magnitudes and direction
                var diff = new Vector3Double(cross.X - B.X, cross.Y - B.Y, cross.Z - B.Z);
                Assert.That(diff.magnitude, Is.LessThan(1e-6), $"B != T x N at u={u}");

                // Ensure helper methods return same vectors
                var t2 = CurveEvaluator.EvaluateTangent(curve, u);
                var n2 = CurveEvaluator.EvaluateNormal(curve, u);
                var b2 = CurveEvaluator.EvaluateBinormal(curve, u);
                var dt = new Vector3Double(t2.X - T.X, t2.Y - T.Y, t2.Z - T.Z);
                var dn = new Vector3Double(n2.X - N.X, n2.Y - N.Y, n2.Z - N.Z);
                var db = new Vector3Double(b2.X - B.X, b2.Y - B.Y, b2.Z - B.Z);
                Assert.That(dt.magnitude, Is.LessThan(1e-12));
                Assert.That(dn.magnitude, Is.LessThan(1e-12));
                Assert.That(db.magnitude, Is.LessThan(1e-12));
            }
        }

        [Test]
        public void FrenetFrame_Line_NormalAndBinormalAreZero_TangentMatches()
        {
            // Straight line from (0,0,0) to (6,8,0)
            int degree = 2;
            double[] knots = { 0, 0, 0, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 4.0, 0.0, 1),
                new ControlPoint(6.0, 8.0, 0.0, 1)
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            double[] sampleUs = { 0.0, 0.25, 0.5, 0.75, 0.999999 };//u=1.0 case is unstable
            const double tol = 1e-8;
            var expectedT = new Vector3Double(3.0 / 5.0, 4.0 / 5.0, 0.0); // normalized (6,8,0)

            foreach (var u in sampleUs)
            {
                (var t, var n, var b) = CurveEvaluator.EvaluateFrenetFrame(curve, u);

                // Tangent should match expected normalized direction
                Assert.That(Math.Abs(t.X - expectedT.X), Is.LessThan(tol));
                Assert.That(Math.Abs(t.Y - expectedT.Y), Is.LessThan(tol));
                Assert.That(Math.Abs(t.Z - expectedT.Z), Is.LessThan(tol));

                // For a straight line, second derivative is zero -> normal & binormal should be zero vectors
                Assert.That(n.magnitude, Is.EqualTo(0.0).Within(tol));
                Assert.That(b.magnitude, Is.EqualTo(0.0).Within(tol));

                // Ensure helper methods return same vectors
                var t2 = CurveEvaluator.EvaluateTangent(curve, u);
                var n2 = CurveEvaluator.EvaluateNormal(curve, u);
                var b2 = CurveEvaluator.EvaluateBinormal(curve, u);
                var dt = new Vector3Double(t2.X - t.X, t2.Y - t.Y, t2.Z - t.Z);
                var dn = new Vector3Double(n2.X - n.X, n2.Y - n.Y, n2.Z - n.Z);
                var db = new Vector3Double(b2.X - b.X, b2.Y - b.Y, b2.Z - b.Z);
                Assert.That(dt.magnitude, Is.LessThan(1e-12));
                Assert.That(dn.magnitude, Is.LessThan(1e-12));
                Assert.That(db.magnitude, Is.LessThan(1e-12));
            }
        }
    }
}
