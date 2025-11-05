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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints1);

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
                Console.WriteLine($"Evaluating at u={u} expected={expected} pt={pt}");

                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints2);

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
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

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
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

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
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

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
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

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
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

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
                Console.WriteLine($"u={u} => pt=({pt.x}, {pt.y}, {pt.z})");
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.0001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.0001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.0001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

            double expectedLength = Math.PI * 2;
            double length = curve.GetLength();

            Assert.That(length, Is.EqualTo(expectedLength).Within(0.001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

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
            double[] knots = { 0, 0.143, 0.286, 0.429, 0.571, 0.714, 0.857, 1 };
            ControlPoint[] controlPoints = {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 0.0, 0.0, 1),
            new ControlPoint(2.0, 0.0, 0.0, 1),
            new ControlPoint(3.0, 3.0, 0.0, 1),
            new ControlPoint(4.0, 3.0, 0.0, 1),
            new ControlPoint(5.0, 3.0, 0.0, 1)
        };
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

            var samples = new (double u, Vector3Double expected)[] {
                (0.150, new Vector3Double(0.049, 0.000000, 0.000000)),
                (0.250, new Vector3Double(0.748, 0.000000, 0.000000)),
                (0.500, new Vector3Double(2.500, 1.500000, 0.000000)),
                (0.750, new Vector3Double(4.252, 3.000000, 0.000000)),
                (0.850, new Vector3Double(4.951, 3.000000, 0.000000))
            };

            foreach (var (u, expected) in samples)
            {
                var pt = CurveEvaluator.Evaluate(curve, u);
                Console.WriteLine($"u={u}, pt=({pt.x}, {pt.y}, {pt.z})");
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.001));
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);


            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };
            const double h = 1e-10;
            foreach (var u in samplePoints)
            {
                var derivVal = CurveEvaluator.EvaluateFirstDerivative(curve,u);
                //finite difference approximation
                var evalPt = CurveEvaluator.Evaluate(curve, u);
                var evalPt2 = CurveEvaluator.Evaluate(curve, u+h);
                //Console.WriteLine($"u={u}: evalPt=({evalPt.x}, {evalPt.y}, {evalPt.z}), evalPt2=({evalPt2.x}, {evalPt2.y}, {evalPt2.z})");
                var evalDerivPt = new Vector3Double(
                    (evalPt2.x - evalPt.x)/h,
                    (evalPt2.y - evalPt.y)/h,
                    (evalPt2.z - evalPt.z)/h
                );
                Console.WriteLine($"u={u}: derivVal=({derivVal.X}, {derivVal.Y}, {derivVal.Z}) vs finite dif=({evalDerivPt.X}, {evalDerivPt.Y}, {evalDerivPt.Z})");
                Assert.That(derivVal.X, Is.EqualTo(evalDerivPt.X).Within(0.001));
                Assert.That(derivVal.Y, Is.EqualTo(evalDerivPt.Y).Within(0.001));
                Assert.That(derivVal.Z, Is.EqualTo(evalDerivPt.Z).Within(0.001));
            }
            
        }

        [Test]
        public void NurbsCurveDerivationTestB()
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
            var curve = new NurbsCurve(degree, new KnotVector(knots), controlPoints);

            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };

            const double h = 1e-10;
            foreach (var u in samplePoints)
            {
                var derivVal = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                //finite difference approximation
                var evalPt = CurveEvaluator.Evaluate(curve, u);
                var evalPt2 = CurveEvaluator.Evaluate(curve, u + h);
                var evalDerivPt = new Vector3Double(
                    (evalPt2.x - evalPt.x) / h,
                    (evalPt2.y - evalPt.y) / h,
                    (evalPt2.z - evalPt.z) / h
                );
                Console.WriteLine($"u={u}: derivVal=({derivVal.X}, {derivVal.Y}, {derivVal.Z}) vs finite dif=({evalDerivPt.X}, {evalDerivPt.Y}, {evalDerivPt.Z})");
                Assert.That(derivVal.X, Is.EqualTo(evalDerivPt.X).Within(0.001));
                Assert.That(derivVal.Y, Is.EqualTo(evalDerivPt.Y).Within(0.001));
                Assert.That(derivVal.Z, Is.EqualTo(evalDerivPt.Z).Within(0.001));
            }
        }


    }
}
