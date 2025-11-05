using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;
using NurbsSharp.Tesselation;
using NUnit.Framework;

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
            double area = SurfaceEvaluator.SurfaceArea(surface, 0, 1, 0, 1, 0.01);
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
            Assert.That(area, Is.EqualTo(10.05).Within(0.01));
        }
    }
}
