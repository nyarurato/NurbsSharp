using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;

namespace UnitTests.Evaluation
{
    internal class SurfaceEvaluationTest
    {

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

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

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

                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
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

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

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

                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
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

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

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

                Assert.That(expected.X, Is.EqualTo(pt.X).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.Y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.Z).Within(0.000001));
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
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            double area = SurfaceEvaluator.SurfaceArea(surface, 0, 1, 0, 1);
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
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            double area = SurfaceEvaluator.SurfaceArea(surface, 0, 1, 0, 1);
            Assert.That(area, Is.EqualTo(10.05).Within(0.01));
            area = SurfaceEvaluator.SurfaceArea(surface, 0, 0.5, 0, 0.25);
        }

        [Test]
        public void SurfaceAreaTestC()
        {
            int degreeU = 3;
            int degreeV = 3;
            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1),
                new ControlPoint(2.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 0.0, 0.0, 1)
            };

            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 0.0, 1),
                new ControlPoint(2.0, 1.0, 0.0, 1),
                new ControlPoint(3.0, 1.0, 0.0, 1)
            };

            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(0.0, 2.0, 0.0, 1),
                new ControlPoint(1.0, 2.0, 0.0, 1),
                new ControlPoint(2.0, 2.0, 0.0, 1),
                new ControlPoint(3.0, 2.0, 0.0, 1)
            };

            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(0.0, 3.0, 0.0, 1),
                new ControlPoint(1.0, 3.0, 0.0, 1),
                new ControlPoint(2.0, 3.0, 0.0, 1),
                new ControlPoint(3.0, 3.0, 0.0, 1)
            };
            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            double area = SurfaceEvaluator.SurfaceArea(surface, 0, 1, 0, 1);
            Assert.That(area, Is.EqualTo(9.0).Within(0.000001));
            area = SurfaceEvaluator.SurfaceArea(surface, 0, 0.5, 0, 0.5);
            Assert.That(area, Is.EqualTo(9.0 / 4).Within(0.000001));
            area = SurfaceEvaluator.SurfaceArea(surface, 0.5, 1, 0.5, 1);
            Assert.That(area, Is.EqualTo(9.0 / 4).Within(0.000001));
        }

        [Test]
        public void SurfaceAreaTestD()
        {
            //Circle surface
            int degreeU = 3;
            int degreeV = 3;
            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1),
                new ControlPoint(2.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 0.0, 0.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 2.5, 1),
                new ControlPoint(2.0, 1.0, 2.0, 1),
                new ControlPoint(3.0, 1.0, 0.0, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(0.0, 2.0, 0.0, 1),
                new ControlPoint(1.0, 2.0, 2.5, 1),
                new ControlPoint(2.0, 2.0, 2.0, 1),
                new ControlPoint(3.0, 2.0, 0.0, 1)
            };
            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(0.0, 3.0, 0.0, 1),
                new ControlPoint(1.0, 3.0, 0.0, 1),
                new ControlPoint(2.0, 3.0, 0.0, 1),
                new ControlPoint(3.0, 3.0, 0.0, 1)
            };
            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            double area = SurfaceEvaluator.SurfaceArea(surface, 0, 1, 0, 1);

            Assert.That(area, Is.EqualTo(12.62).Within(0.01));

        }

        [Test]
        public void SurfaceDerivativeTestA()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
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
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };

            const double h = 1e-6;
            foreach (var (u, v) in samples)
            {
                var derivVal = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                //finite difference approximation
                var deriv1 = SurfaceEvaluator.Evaluate(surface, u, v);
                var deriv1bu = SurfaceEvaluator.Evaluate(surface, u + h, v);
                var derib1bv = SurfaceEvaluator.Evaluate(surface, u, v + h);

                var evalDeriv2PtU = new Vector3Double(
                    (deriv1bu.X - deriv1.X) / h,
                    (deriv1bu.Y - deriv1.Y) / h,
                    (deriv1bu.Z - deriv1.Z) / h
                );
                var evalDeriv2PtV = new Vector3Double(
                    (derib1bv.X - deriv1.X) / h,
                    (derib1bv.Y - deriv1.Y) / h,
                    (derib1bv.Z - deriv1.Z) / h
                );

                Assert.That(derivVal.u_deriv.X, Is.EqualTo(evalDeriv2PtU.X).Within(0.01));
                Assert.That(derivVal.u_deriv.Y, Is.EqualTo(evalDeriv2PtU.Y).Within(0.01));
                Assert.That(derivVal.u_deriv.Z, Is.EqualTo(evalDeriv2PtU.Z).Within(0.01));

                Assert.That(derivVal.v_deriv.X, Is.EqualTo(evalDeriv2PtV.X).Within(0.01));
                Assert.That(derivVal.v_deriv.Y, Is.EqualTo(evalDeriv2PtV.Y).Within(0.01));
                Assert.That(derivVal.v_deriv.Z, Is.EqualTo(evalDeriv2PtV.Z).Within(0.01));
            }
        }

        [Test]
        public void SurfaceDerivativeTestB()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
            int degreeU = 2;
            int degreeV = 3;

            double[] knotsU = { 0, 0, 0, 0.5, 1, 1, 1 };
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
                new ControlPoint(4.0, 2.0, 0.5, 1),
                new ControlPoint(4.0, 2.0, 2.5, 1),
                new ControlPoint(4.0, 2.0, 5.0, 1),
                new ControlPoint(4.0, 2.0, 7.0, 1)

            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };

            const double h = 1e-6;
            foreach (var (u, v) in samples)
            {
                var derivVal = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                //finite difference approximation
                var evalpt = SurfaceEvaluator.Evaluate(surface, u, v);
                var evalpt_du = SurfaceEvaluator.Evaluate(surface, u + h, v);
                var evalpt_dv = SurfaceEvaluator.Evaluate(surface, u, v + h);

                var evalDeriv2PtU = new Vector3Double(
                    (evalpt_du.X - evalpt.X) / h,
                    (evalpt_du.Y - evalpt.Y) / h,
                    (evalpt_du.Z - evalpt.Z) / h
                );
                var evalDeriv2PtV = new Vector3Double(
                    (evalpt_dv.X - evalpt.X) / h,
                    (evalpt_dv.Y - evalpt.Y) / h,
                    (evalpt_dv.Z - evalpt.Z) / h
                );


                Assert.That(derivVal.u_deriv.X, Is.EqualTo(evalDeriv2PtU.X).Within(0.01));
                Assert.That(derivVal.u_deriv.Y, Is.EqualTo(evalDeriv2PtU.Y).Within(0.01));
                Assert.That(derivVal.u_deriv.Z, Is.EqualTo(evalDeriv2PtU.Z).Within(0.01));

                Assert.That(derivVal.v_deriv.X, Is.EqualTo(evalDeriv2PtV.X).Within(0.01));
                Assert.That(derivVal.v_deriv.Y, Is.EqualTo(evalDeriv2PtV.Y).Within(0.01));
                Assert.That(derivVal.v_deriv.Z, Is.EqualTo(evalDeriv2PtV.Z).Within(0.01));
            }
        }

        [Test]
        public void SurfaceSecondDerivativeTestA()
        {
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

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            const double h = 1e-6;
            foreach (var (u, v) in samples)
            {
                var derivVal = SurfaceEvaluator.EvaluateSecondDerivative(surface, u, v);
                //finite difference approximation
                var evalpt = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                var evalpt_du = SurfaceEvaluator.EvaluateFirstDerivative(surface, u + h, v);
                var evalpt_dv = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v + h);
                var evalpt_duv = SurfaceEvaluator.EvaluateFirstDerivative(surface, u + h, v + h);
                var evalDeriv2PtUU = new Vector3Double(
                    (evalpt_du.u_deriv.X - evalpt.u_deriv.X) / h,
                    (evalpt_du.u_deriv.Y - evalpt.u_deriv.Y) / h,
                    (evalpt_du.u_deriv.Z - evalpt.u_deriv.Z) / h
                );
                var evalDeriv2PtVV = new Vector3Double(
                    (evalpt_dv.v_deriv.X - evalpt.v_deriv.X) / h,
                    (evalpt_dv.v_deriv.Y - evalpt.v_deriv.Y) / h,
                    (evalpt_dv.v_deriv.Z - evalpt.v_deriv.Z) / h
                );
                var evalDeriv2PtUV_from_du = new Vector3Double(
                    (evalpt_du.v_deriv.X - evalpt.v_deriv.X) / h,
                    (evalpt_du.v_deriv.Y - evalpt.v_deriv.Y) / h,
                    (evalpt_du.v_deriv.Z - evalpt.v_deriv.Z) / h
                );
                // option2: ∂/∂v (u-deriv) ≈ (u_deriv(u,v+h) - u_deriv(u,v)) / h
                var evalDeriv2PtUV_from_dv = new Vector3Double(
                    (evalpt_dv.u_deriv.X - evalpt.u_deriv.X) / h,
                    (evalpt_dv.u_deriv.Y - evalpt.u_deriv.Y) / h,
                    (evalpt_dv.u_deriv.Z - evalpt.u_deriv.Z) / h
                );
                // average the two finite-difference estimates to reduce asymmetry
                var evalDeriv2PtUV = new Vector3Double(
                    0.5 * (evalDeriv2PtUV_from_du.X + evalDeriv2PtUV_from_dv.X),
                    0.5 * (evalDeriv2PtUV_from_du.Y + evalDeriv2PtUV_from_dv.Y),
                    0.5 * (evalDeriv2PtUV_from_du.Z + evalDeriv2PtUV_from_dv.Z)
                );


                Assert.That(derivVal.uu_deriv.X, Is.EqualTo(evalDeriv2PtUU.X).Within(0.1));
                Assert.That(derivVal.uu_deriv.Y, Is.EqualTo(evalDeriv2PtUU.Y).Within(0.1));
                Assert.That(derivVal.uu_deriv.Z, Is.EqualTo(evalDeriv2PtUU.Z).Within(0.1));
                Assert.That(derivVal.vv_deriv.X, Is.EqualTo(evalDeriv2PtVV.X).Within(0.1));
                Assert.That(derivVal.vv_deriv.Y, Is.EqualTo(evalDeriv2PtVV.Y).Within(0.1));
                Assert.That(derivVal.vv_deriv.Z, Is.EqualTo(evalDeriv2PtVV.Z).Within(0.1));

                Assert.That(derivVal.uv_deriv.X, Is.EqualTo(evalDeriv2PtUV.X).Within(0.1));
                Assert.That(derivVal.uv_deriv.Y, Is.EqualTo(evalDeriv2PtUV.Y).Within(0.1));
                Assert.That(derivVal.uv_deriv.Z, Is.EqualTo(evalDeriv2PtUV.Z).Within(0.1));
            }
        }

        [Test]
        public void SurfaceSecondDerivativeTestB()
        {
            //Plane surface
            int degreeU = 2;
            int degreeV = 2;
            double[] knotsU = { 0, 0, 0, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 1, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1),
                new ControlPoint(2.0, 0.0, 0.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 0.0, 1),
                new ControlPoint(2.0, 1.0, 0.0, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(0.0, 2.0, 0.0, 1),
                new ControlPoint(1.0, 2.0, 0.0, 1),
                new ControlPoint(2.0, 2.0, 0.0, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            foreach (var (u, v) in samples)
            {
                var derivVal = SurfaceEvaluator.EvaluateSecondDerivative(surface, u, v);
                Assert.That(derivVal.uu_deriv.X, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.uu_deriv.Y, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.uu_deriv.Z, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.vv_deriv.X, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.vv_deriv.Y, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.vv_deriv.Z, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.uv_deriv.X, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.uv_deriv.Y, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(derivVal.uv_deriv.Z, Is.EqualTo(0.0).Within(0.000001));
            }
        }

        [Test]
        public void SurfaceSecondDerivativeTestC()
        {
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = { 0, 0, 0.5, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 0, 1),
                new ControlPoint(1 ,  1, 0, 0.70710678),
                new ControlPoint(0 ,  1, 0, 1),
                new ControlPoint(-1,  1, 0, 0.70710678),
                new ControlPoint(-1,  0, 0, 1),
                new ControlPoint(-1, -1, 0, 0.70710678),
                new ControlPoint(0 , -1, 0, 1),
                new ControlPoint(1 , -1, 0, 0.70710678),
                new ControlPoint(1 ,  0, 0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 1, 1),
                new ControlPoint(1 ,  1, 1, 0.70710678),
                new ControlPoint(0 ,  1, 1, 1),
                new ControlPoint(-1,  1, 1, 0.70710678),
                new ControlPoint(-1,  0, 1, 1),
                new ControlPoint(-1, -1, 1, 0.70710678),
                new ControlPoint(0 , -1, 1, 1),
                new ControlPoint(1 , -1, 1, 0.70710678),
                new ControlPoint(1 ,  0, 1, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 2, 1),
                new ControlPoint(1 ,  1, 2, 0.70710678),
                new ControlPoint(0 ,  1, 2, 1),
                new ControlPoint(-1,  1, 2, 0.70710678),
                new ControlPoint(-1,  0, 2, 1),
                new ControlPoint(-1, -1, 2, 0.70710678),
                new ControlPoint(0 , -1, 2, 1),
                new ControlPoint(1 , -1, 2, 0.70710678),
                new ControlPoint(1 ,  0, 2, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            var R = 1.0;
            foreach (var (u, v) in samples)
            {
                var deriv2Val = SurfaceEvaluator.EvaluateSecondDerivative(surface, u, v);
                var derivVal = SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);
                var curvature = Vector3Double.Cross(derivVal.v_deriv, deriv2Val.vv_deriv).magnitude / Math.Pow(derivVal.v_deriv.magnitude, 3);
                //For a cylinder, all second derivatives should be zero except vv_deriv.Z
                Assert.That(deriv2Val.uu_deriv.X, Is.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.uu_deriv.Y, Is.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.uu_deriv.Z, Is.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.uv_deriv.X, Is.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.uv_deriv.Y, Is.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.vv_deriv.X, Is.Not.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.vv_deriv.Y, Is.Not.EqualTo(0.0).Within(0.0001));
                Assert.That(deriv2Val.vv_deriv.Z, Is.EqualTo(0.0).Within(0.0001));// cylinder axis direction

                Assert.That(deriv2Val.vv_deriv.X, !Is.EqualTo(0.0).Within(0.0001));


                Assert.That(curvature, Is.EqualTo(1 / R).Within(0.0001));

            }
        }

        [Test]
        public void SurfaceNormalTestA()
        {
            //Plane surface
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
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 0.0, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            foreach (var (u, v) in samples)
            {
                var normal = SurfaceEvaluator.EvaluateNormal(surface, u, v);
                var expected = new Vector3Double(0.0, 0.0, 1.0);
                Assert.That(expected.X, Is.EqualTo(Math.Abs(normal.X)).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(Math.Abs(normal.Y)).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(Math.Abs(normal.Z)).Within(0.000001));
            }
        }

        [Test]
        public void SurfaceNormalTestB()
        {
            //Slanted plane surface
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
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 1.0, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            var expected = new Vector3Double(-1, 0, 1).normalized;
            foreach (var (u, v) in samples)
            {
                var normal = SurfaceEvaluator.EvaluateNormal(surface, u, v);
                Assert.That(normal.magnitude, Is.EqualTo(expected.magnitude));
                Assert.That(Math.Abs(expected.X), Is.EqualTo(Math.Abs(normal.X)).Within(0.000001));
                Assert.That(Math.Abs(expected.Y), Is.EqualTo(Math.Abs(normal.Y)).Within(0.000001));
                Assert.That(Math.Abs(expected.Z), Is.EqualTo(Math.Abs(normal.Z)).Within(0.000001));
            }
        }

        [Test]
        public void SurfaceNormalTestC()
        {
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = { 0, 0, 0.5, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 0, 1),
                new ControlPoint(1 ,  1, 0, 0.70710678),
                new ControlPoint(0 ,  1, 0, 1),
                new ControlPoint(-1,  1, 0, 0.70710678),
                new ControlPoint(-1,  0, 0, 1),
                new ControlPoint(-1, -1, 0, 0.70710678),
                new ControlPoint(0 , -1, 0, 1),
                new ControlPoint(1 , -1, 0, 0.70710678),
                new ControlPoint(1 ,  0, 0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 1, 1),
                new ControlPoint(1 ,  1, 1, 0.70710678),
                new ControlPoint(0 ,  1, 1, 1),
                new ControlPoint(-1,  1, 1, 0.70710678),
                new ControlPoint(-1,  0, 1, 1),
                new ControlPoint(-1, -1, 1, 0.70710678),
                new ControlPoint(0 , -1, 1, 1),
                new ControlPoint(1 , -1, 1, 0.70710678),
                new ControlPoint(1 ,  0, 1, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 2, 1),
                new ControlPoint(1 ,  1, 2, 0.70710678),
                new ControlPoint(0 ,  1, 2, 1),
                new ControlPoint(-1,  1, 2, 0.70710678),
                new ControlPoint(-1,  0, 2, 1),
                new ControlPoint(-1, -1, 2, 0.70710678),
                new ControlPoint(0 , -1, 2, 1),
                new ControlPoint(1 , -1, 2, 0.70710678),
                new ControlPoint(1 ,  0, 2, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            foreach (var (u, v) in samples)
            {
                var normal = SurfaceEvaluator.EvaluateNormal(surface, u, v);

                var evalpt = SurfaceEvaluator.Evaluate(surface, u, v);
                var expected = new Vector3Double(evalpt.X, evalpt.Y, 0).normalized;
                Assert.That(normal.magnitude, Is.EqualTo(expected.magnitude).Within(0.0001));
                Assert.That(Math.Abs(expected.X), Is.EqualTo(Math.Abs(normal.X)).Within(0.0001));
                Assert.That(Math.Abs(expected.Y), Is.EqualTo(Math.Abs(normal.Y)).Within(0.0001));
                Assert.That(Math.Abs(expected.Z), Is.EqualTo(Math.Abs(normal.Z)).Within(0.0001));


            }
        }
        [Test]
        public void SurfaceTangentTestA()
        {
            //Plane surface
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
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 0.0, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            foreach (var (u, v) in samples)
            {
                var (tangentU, tangentV) = SurfaceEvaluator.EvaluateTangents(surface, u, v);
                var expectedTangentU = new Vector3Double(0.0, 1.0, 0.0);
                var expectedTangentV = new Vector3Double(1.0, 0.0, 0.0);
                Assert.That(expectedTangentU.X, Is.EqualTo(tangentU.X).Within(0.000001));
                Assert.That(expectedTangentU.Y, Is.EqualTo(tangentU.Y).Within(0.000001));
                Assert.That(expectedTangentU.Z, Is.EqualTo(tangentU.Z).Within(0.000001));

            }
        }

        [Test]
        public void SurfaceTangentTestB()
        {
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = { 0, 0, 0.5, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 0, 1),
                new ControlPoint(1 ,  1, 0, 0.70710678),
                new ControlPoint(0 ,  1, 0, 1),
                new ControlPoint(-1,  1, 0, 0.70710678),
                new ControlPoint(-1,  0, 0, 1),
                new ControlPoint(-1, -1, 0, 0.70710678),
                new ControlPoint(0 , -1, 0, 1),
                new ControlPoint(1 , -1, 0, 0.70710678),
                new ControlPoint(1 ,  0, 0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 1, 1),
                new ControlPoint(1 ,  1, 1, 0.70710678),
                new ControlPoint(0 ,  1, 1, 1),
                new ControlPoint(-1,  1, 1, 0.70710678),
                new ControlPoint(-1,  0, 1, 1),
                new ControlPoint(-1, -1, 1, 0.70710678),
                new ControlPoint(0 , -1, 1, 1),
                new ControlPoint(1 , -1, 1, 0.70710678),
                new ControlPoint(1 ,  0, 1, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(1 ,  0, 2, 1),
                new ControlPoint(1 ,  1, 2, 0.70710678),
                new ControlPoint(0 ,  1, 2, 1),
                new ControlPoint(-1,  1, 2, 0.70710678),
                new ControlPoint(-1,  0, 2, 1),
                new ControlPoint(-1, -1, 2, 0.70710678),
                new ControlPoint(0 , -1, 2, 1),
                new ControlPoint(1 , -1, 2, 0.70710678),
                new ControlPoint(1 ,  0, 2, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            foreach (var (u, v) in samples)
            {
                var (tangentU, tangentV) = SurfaceEvaluator.EvaluateTangents(surface, u, v);
                var evalpt = SurfaceEvaluator.Evaluate(surface, u, v);
                var expectedTangentU = new Vector3Double(0, 0, 1);
                var expectedTangentV = new Vector3Double(-evalpt.Y, evalpt.X, 0).normalized;// tangent along the circular direction
                Assert.That(tangentU.magnitude, Is.EqualTo(expectedTangentU.magnitude).Within(0.0001));
                Assert.That(Math.Abs(expectedTangentU.X), Is.EqualTo(Math.Abs(tangentU.X)).Within(0.0001));
                Assert.That(Math.Abs(expectedTangentU.Y), Is.EqualTo(Math.Abs(tangentU.Y)).Within(0.0001));
                Assert.That(Math.Abs(expectedTangentU.Z), Is.EqualTo(Math.Abs(tangentU.Z)).Within(0.0001));
                Assert.That(tangentV.magnitude, Is.EqualTo(expectedTangentV.magnitude).Within(0.0001));
                Assert.That(Math.Abs(expectedTangentV.X), Is.EqualTo(Math.Abs(tangentV.X)).Within(0.0001));
                Assert.That(Math.Abs(expectedTangentV.Y), Is.EqualTo(Math.Abs(tangentV.Y)).Within(0.0001));
                Assert.That(Math.Abs(expectedTangentV.Z), Is.EqualTo(Math.Abs(tangentV.Z)).Within(0.0001));
            }
        }

        [Test]
        public void SurfaceCurvatureTestA()
        {
            var R = 3.0;
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = { 0, 0, 0.5, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(R ,  0, 0, 1),
                new ControlPoint(R ,  R, 0, 0.70710678),
                new ControlPoint(0 ,  R, 0, 1),
                new ControlPoint(-R,  R, 0, 0.70710678),
                new ControlPoint(-R,  0, 0, 1),
                new ControlPoint(-R, -R, 0, 0.70710678),
                new ControlPoint(0 , -R, 0, 1),
                new ControlPoint(R , -R, 0, 0.70710678),
                new ControlPoint(R ,  0, 0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(R ,  0, 1, 1),
                new ControlPoint(R ,  R, 1, 0.70710678),
                new ControlPoint(0 ,  R, 1, 1),
                new ControlPoint(-R,  R, 1, 0.70710678),
                new ControlPoint(-R,  0, 1, 1),
                new ControlPoint(-R, -R, 1, 0.70710678),
                new ControlPoint(0 , -R, 1, 1),
                new ControlPoint(R , -R, 1, 0.70710678),
                new ControlPoint(R ,  0, 1, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(R ,  0, 2, 1),
                new ControlPoint(R ,  R, 2, 0.70710678),
                new ControlPoint(0 ,  R, 2, 1),
                new ControlPoint(-R,  R, 2, 0.70710678),
                new ControlPoint(-R,  0, 2, 1),
                new ControlPoint(-R, -R, 2, 0.70710678),
                new ControlPoint(0 , -R, 2, 1),
                new ControlPoint(R , -R, 2, 0.70710678),
                new ControlPoint(R ,  0, 2, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.15, 0.5),
                (0.0001, 0.0001),
                (0.5, 0.00001),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };


            foreach (var (u, v) in samples)
            {
                (double k1, double k2) = SurfaceEvaluator.EvaluatePrincipalCurvatures(surface, u, v);
                (double H, double K) = SurfaceEvaluator.EvaluateMeanAndGaussianCurvatures(surface, u, v);
                //For a cylinder, one principal curvature should be 0, the other should be 1/R
                Assert.That(k1, Is.EqualTo(0.0).Within(0.0001).Or.EqualTo(1 / R).Within(0.0001));
                Assert.That(k2, Is.EqualTo(0.0).Within(0.0001).Or.EqualTo(1 / R).Within(0.0001));
                Assert.That(Math.Abs(k1), Is.Not.EqualTo(Math.Abs(k2)).Within(0.0001));
                //Mean curvature H = (k1 + k2) / 2
                Assert.That(H, Is.EqualTo((0 + 1 / R) / 2).Within(0.0001));
                //Gaussian curvature K = k1 * k2
                Assert.That(K, Is.EqualTo(0.0).Within(0.0001));
            }
        }

        [Test]
        public void SurfaceCurvatureTestB()
        {
            //Plane surface
            int degreeU = 2;
            int degreeV = 2;
            double[] knotsU = { 0, 0, 0, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 1, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1),
                new ControlPoint(2.0, 0.0, 0.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.0, 1),
                new ControlPoint(1.0, 1.0, 0.0, 1),
                new ControlPoint(2.0, 1.0, 0.0, 1)
            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(0.0, 2.0, 0.0, 1),
                new ControlPoint(1.0, 2.0, 0.0, 1),
                new ControlPoint(2.0, 2.0, 0.0, 1)
            };
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            var samples = new (double u, double v)[] {
                (0.0, 0.0),
                (0.99, 0.0),
                (0.0, 0.99),
                (0.99, 0.99),
                (0.15, 0.5),
                (0.0001, 0.0001),
                (0.5, 0.00001),
                (0.5, 0.5),
                (0.7, 0.7),
                (0.1, 0.4)
            };
            foreach (var (u, v) in samples)
            {
                (double k1, double k2) = SurfaceEvaluator.EvaluatePrincipalCurvatures(surface, u, v);
                (double H, double K) = SurfaceEvaluator.EvaluateMeanAndGaussianCurvatures(surface, u, v);
                //For a plane, both principal curvatures should be 0
                Assert.That(k1, Is.EqualTo(0.0).Within(0.000001));
                Assert.That(k2, Is.EqualTo(0.0).Within(0.000001));
                //Mean curvature H = (k1 + k2) / 2
                Assert.That(H, Is.EqualTo(0.0).Within(0.000001));
                //Gaussian curvature K = k1 * k2
                Assert.That(K, Is.EqualTo(0.0).Within(0.000001));

            }
        } 
    }
}
