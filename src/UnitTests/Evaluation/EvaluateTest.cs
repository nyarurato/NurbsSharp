using System;
using System.Collections.Generic;

using NUnit.Framework;
using NUnit.Framework.Legacy;

using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;


namespace UnitTests.Evaluation
{
    [TestFixture]
    public class EvaluateTest
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
        public void CubicNurbsCurve_GetLength_DatasetA()
        {
            // Circle R=1
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75,0.75, 1, 1, 1 };
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
        public void NurbsSurfaceTestA()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = { 0, 0, 1, 1 };
            double[] knotsV = { 0, 0, 1, 1 };

            ControlPoint[][] controlPoints = new ControlPoint[2][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1)
            };

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU), new KnotVector(knotsV), controlPoints);

            var samples = new (double u, double v, Vector3Double expected)[] {
                (0.000, 0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (1.000, 0.000, new Vector3Double(0.000000, 1.000000, 0.500000)),
                (0.000, 1.000, new Vector3Double(1.000000, 0.000000, 1.000000)),
                (1.000, 1.000, new Vector3Double(1.000000, 1.000000, 1.500000)),
                (0.500, 0.500, new Vector3Double(0.500000, 0.500000, 0.750000)),
                (0.250, 0.750, new Vector3Double(0.750000, 0.250000, 0.875000)),
                (0.750, 0.250, new Vector3Double(0.250000, 0.750000, 0.625000)),
                (0.10, 0.10, new Vector3Double(0.10000, 0.10000, 0.15000)),
                (0.90, 0.90, new Vector3Double(0.90000, 0.90000, 1.35000)),

            };

            foreach (var (u, v, expected) in samples)
            {
                var pt = SurfaceEvaluator.Evaluate(surface, u, v);
                Console.WriteLine($"Evaluating at u={u}, v={v} expected={expected} pt={pt}");

                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
            }
        }

        [Test]
        public void NurbsSurfaceTestB()
        {
            // NURBS surface (degree 2 in both directions)
            int degreeU = 2;
            int degreeV = 2;

            double[] knotsU = { 0, 0, 0, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 1, 1, 1 };

            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1),
                new ControlPoint(2.0, 0.0, 3.0, 1)

            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1)

            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(3.0, 1.0, 0.5, 1),
                new ControlPoint(3.0, 1.0, 1.5, 1),
                new ControlPoint(3.0, 1.0, 5.0, 1)

            };

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU), new KnotVector(knotsV), controlPoints);

            var samples = new (double u, double v, Vector3Double expected)[] {
                (0.000, 0.000, new Vector3Double(0.0, 0.0, 0.0)),
                (1.000, 0.000, new Vector3Double(3.0, 1.0, 0.5)),
                (0.000, 1.000, new Vector3Double(2.0, 0.0, 3.0)),
                (1.000, 1.000, new Vector3Double(3.0, 1.0, 5.0)),
                (0.500, 0.500, new Vector3Double(1.5, 0.75, 1.78125)),
                (0.250, 0.750, new Vector3Double(1.59375, 0.4375, 2.439453125)),
                (0.750, 0.250, new Vector3Double(1.90625, 0.9375, 1.095703125)),
                (0.10, 0.10, new Vector3Double(0.2280000, 0.190000, 0.3060500)),
                (0.90, 0.90, new Vector3Double(2.772000, 0.99, 4.162050000))

            };

            foreach (var (u, v, expected) in samples)
            {
                var pt = SurfaceEvaluator.Evaluate(surface, u, v);
                Console.WriteLine($"Evaluating at u={u}, v={v} expected={expected} pt={pt}");

                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
            }
        }

        [Test]
        public void NurbsSurfaceTestC()
        {
            // NURBS surface (degree 3 in both directions)
            int degreeU = 3;
            int degreeV = 3;

            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1),
                new ControlPoint(2.0, 0.0, 3.0, 1),
                new ControlPoint(3.0, 0.0, 3.0, 1)

            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1),
                new ControlPoint(3.0, 1.0, 3.0, 1)

            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(3.0, 1.0, 0.5, 1),
                new ControlPoint(3.0, 1.0, 1.5, 1),
                new ControlPoint(3.0, 1.0, 5.0, 1),
                new ControlPoint(3.0, 1.0, 6.0, 1)

            };
            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(3.0, 2.0, 0.5, 1),
                new ControlPoint(3.0, 2.0, 1.5, 1),
                new ControlPoint(3.0, 2.0, 5.0, 1),
                new ControlPoint(3.0, 2.0, 7.0, 1)

            };

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU), new KnotVector(knotsV), controlPoints);

            var samples = new (double u, double v, Vector3Double expected)[] {
                (0.000, 0.000, new Vector3Double(0.0, 0.0, 0.0)),
                (1.000, 0.000, new Vector3Double(3.0, 2.0, 0.5)),
                (0.000, 1.000, new Vector3Double(3.0, 0.0, 3.0)),
                (1.000, 1.000, new Vector3Double(3.0, 2.0, 7.0)),
                (0.500, 0.500, new Vector3Double(2.25, 1.0, 2.8125)),
                (0.250, 0.750, new Vector3Double(2.3671875, 0.59375, 3.231201171875)),
                (0.750, 0.250, new Vector3Double(2.6484375, 1.40625, 1.609130859375)),
                (0.10, 0.10, new Vector3Double(0.37560000, 0.27200000, 0.46686400)),
                (0.90, 0.90, new Vector3Double(2.9916, 1.728000, 6.093144000))
            };

            foreach (var (u, v, expected) in samples)
            {
                var pt = SurfaceEvaluator.Evaluate(surface, u, v);
                Console.WriteLine($"Evaluating at u={u}, v={v} expected={expected} pt={pt}");

                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
            }
        }

        [Test]
        public void SurfaceAreaTestA()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
            // Rectangle 1x1 elevated to z=1.5
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = { 0, 0, 1, 1 };
            double[] knotsV = { 0, 0, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[2][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 1.5, 1),
                new ControlPoint(1.0, 0.0, 1.5, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU), new KnotVector(knotsV), controlPoints);
            double area = SurfaceEvaluator.SurfaceArea(surface,0,1,0,1,0.01);
            Console.WriteLine($"Surface area: {area}");
            Assert.That(area, Is.EqualTo(1.50).Within(0.000001));
        }

        [Test]
        public void SurfaceAreaTestB()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
            // Rectangle 1x1 elevated same plane
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = { 0, 0, 1, 1 };
            double[] knotsV = { 0, 0, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[2][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(2.0, 1.0, 1.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 2.0, 1.5, 1),
                new ControlPoint(4.0, 6.0, 5, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU), new KnotVector(knotsV), controlPoints);
            double area = SurfaceEvaluator.SurfaceArea(surface, 0, 1, 0, 1, 0.01);
            Console.WriteLine($"Surface area: {area}");
            Assert.That(area, Is.EqualTo(12.562345).Within(0.000001));
        }

        [Test]
        public void NurbsVolumeTestA()
        {
            // Bilinear NURBS volue (degree 1 in both directions)
            int degreeU = 1;
            int degreeV = 1;
            int degreeW = 1;

            double[] knotsU = { 0, 0, 1, 1 };
            double[] knotsV = { 0, 0, 1, 1 };
            double[] knotsW = { 0, 0, 1, 1 };
            ControlPoint[][][] controlPoints = new ControlPoint[2][][];
            controlPoints[0] = new ControlPoint[2][];
            controlPoints[0][0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1)
            };
            controlPoints[0][1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1)
            };
            controlPoints[1] = new ControlPoint[2][];
            controlPoints[1][0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 1.0, 1),
                new ControlPoint(1.0, 0.0, 2.0, 1)
            };
            controlPoints[1][1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 1.5, 1),
                new ControlPoint(1.0, 1.0, 2.5, 1)
            };
            var volume = new NurbsVolume(degreeU, degreeV, degreeW, new KnotVector(knotsU), new KnotVector(knotsV), new KnotVector(knotsW), controlPoints);

            var samples = new (double u, double v, double w, Vector3Double expected)[] {
                (0.000, 0.000, 0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (1.000, 0.000, 0.000, new Vector3Double(0.000000, 0.000000, 1.000000)),
                (0.000, 1.000, 0.000, new Vector3Double(0.000000, 1.000000, 0.500000)),
                (1.000, 1.000, 0.000, new Vector3Double(0.000000, 1.000000, 1.500000)),
                (0.000, 0.000, 1.000, new Vector3Double(1.000000, 0.000000, 1.000000)),
                (1.000, 0.000, 1.000, new Vector3Double(1.000000, 0.000000, 2.000000)),
                (0.000, 1.000, 1.000, new Vector3Double(1.000000, 1.000000, 1.500000)),
                (1.000, 1.000, 1.000, new Vector3Double(1.000000, 1.000000, 2.500000)),
                (0.500, 0.500, 0.500, new Vector3Double(0.500000, 0.50000, 1.25000))
            };

            foreach (var (u, v, w, expected) in samples)
            {
                var pt = VolumeEvaluator.Evaluate(volume, u, v, w);
                Console.WriteLine($"Evaluating at u={u}, v={v}, w={w} expected={expected} pt={pt}");
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
            }
        }
    }
}