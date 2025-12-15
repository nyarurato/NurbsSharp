using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using NurbsSharp.Generation;
using System;
using System.Linq;
using NurbsSharp.Analysis;
using NurbsSharp.Evaluation;

namespace UnitTests.Analysis
{
    [TestFixture]
    public class SurfaceAnalyzerTest
    {

        [Test]
        public void SurfaceAreaTestA()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
            // Rectangle 1x1 elevated to z=1.5
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 0.0, 1.5, 1),
                    new ControlPoint(1.0, 0.0, 1.5, 1)
                ],
            ];
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);
            double area = SurfaceAnalyzer.SurfaceArea(surface, 0, 1, 0, 1);
            Assert.That(area, Is.EqualTo(1.50).Within(0.000001));
        }

        [Test]
        public void SurfaceAreaTestB()
        {
            // Bilinear NURBS surface (degree 1 in both directions)
            // Rectangle 1x1 elevated same plane
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(2.0, 1.0, 1.0, 1)
                ],
                [
                    new ControlPoint(0.0, 2.0, 1.5, 1),
                    new ControlPoint(4.0, 6.0, 5, 1)
                ],
            ];
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            double area = SurfaceAnalyzer.SurfaceArea(surface, 0, 1, 0, 1);
            Assert.That(area, Is.EqualTo(10.05).Within(0.01));
            area = SurfaceAnalyzer.SurfaceArea(surface, 0, 0.5, 0, 0.25);
        }

        [Test]
        public void SurfaceAreaTestC()
        {
            int degreeU = 3;
            int degreeV = 3;
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1),
                    new ControlPoint(2.0, 0.0, 0.0, 1),
                    new ControlPoint(3.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 1.0, 0.0, 1),
                    new ControlPoint(1.0, 1.0, 0.0, 1),
                    new ControlPoint(2.0, 1.0, 0.0, 1),
                    new ControlPoint(3.0, 1.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 2.0, 0.0, 1),
                    new ControlPoint(1.0, 2.0, 0.0, 1),
                    new ControlPoint(2.0, 2.0, 0.0, 1),
                    new ControlPoint(3.0, 2.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 3.0, 0.0, 1),
                    new ControlPoint(1.0, 3.0, 0.0, 1),
                    new ControlPoint(2.0, 3.0, 0.0, 1),
                    new ControlPoint(3.0, 3.0, 0.0, 1)
                ],
            ];
            double[] knotsU = [0, 0, 0, 0, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 1, 1, 1, 1];
            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            double area = SurfaceAnalyzer.SurfaceArea(surface, 0, 1, 0, 1);
            Assert.That(area, Is.EqualTo(9.0).Within(0.000001));
            area = SurfaceAnalyzer.SurfaceArea(surface, 0, 0.5, 0, 0.5);
            Assert.That(area, Is.EqualTo(9.0 / 4).Within(0.000001));
            area = SurfaceAnalyzer.SurfaceArea(surface, 0.5, 1, 0.5, 1);
            Assert.That(area, Is.EqualTo(9.0 / 4).Within(0.000001));
        }

        [Test]
        public void SurfaceAreaTestD()
        {
            //Circle surface
            int degreeU = 3;
            int degreeV = 3;
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1),
                    new ControlPoint(2.0, 0.0, 0.0, 1),
                    new ControlPoint(3.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 1.0, 0.0, 1),
                    new ControlPoint(1.0, 1.0, 2.5, 1),
                    new ControlPoint(2.0, 1.0, 2.0, 1),
                    new ControlPoint(3.0, 1.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 2.0, 0.0, 1),
                    new ControlPoint(1.0, 2.0, 2.5, 1),
                    new ControlPoint(2.0, 2.0, 2.0, 1),
                    new ControlPoint(3.0, 2.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 3.0, 0.0, 1),
                    new ControlPoint(1.0, 3.0, 0.0, 1),
                    new ControlPoint(2.0, 3.0, 0.0, 1),
                    new ControlPoint(3.0, 3.0, 0.0, 1)
                ],
            ];
            double[] knotsU = [0, 0, 0, 0, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 1, 1, 1, 1];

            var surface = new NurbsSurface(degreeU, degreeV, new KnotVector(knotsU, degreeU), new KnotVector(knotsV, degreeV), controlPoints);

            double area = SurfaceAnalyzer.SurfaceArea(surface, 0, 1, 0, 1);

            Assert.That(area, Is.EqualTo(12.62).Within(0.01));

        }


        [Test]
        public void SurfaceNormalTestA()
        {
            //Plane surface
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 1.0, 0.0, 1),
                    new ControlPoint(1.0, 1.0, 0.0, 1)
                ],
            ];
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
                var normal = SurfaceAnalyzer.EvaluateNormal(surface, u, v);
                var expected = new Vector3Double(0.0, 0.0, 1.0);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(expected.X, Is.EqualTo(Math.Abs(normal.X)).Within(0.000001));
                    Assert.That(expected.Y, Is.EqualTo(Math.Abs(normal.Y)).Within(0.000001));
                    Assert.That(expected.Z, Is.EqualTo(Math.Abs(normal.Z)).Within(0.000001));
                }
            }
        }

        [Test]
        public void SurfaceNormalTestB()
        {
            //Slanted plane surface
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 1.0, 1)
                ],
                [
                    new ControlPoint(0.0, 1.0, 0.0, 1),
                    new ControlPoint(1.0, 1.0, 1.0, 1)
                ],
            ];
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
                var normal = SurfaceAnalyzer.EvaluateNormal(surface, u, v);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(normal.magnitude, Is.EqualTo(expected.magnitude));
                    Assert.That(Math.Abs(expected.X), Is.EqualTo(Math.Abs(normal.X)).Within(0.000001));
                    Assert.That(Math.Abs(expected.Y), Is.EqualTo(Math.Abs(normal.Y)).Within(0.000001));
                    Assert.That(Math.Abs(expected.Z), Is.EqualTo(Math.Abs(normal.Z)).Within(0.000001));
                }
            }
        }

        [Test]
        public void SurfaceNormalTestC()
        {
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = [0, 0, 0.5, 1, 1];
            double[] knotsV = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(1 ,  0, 0, 1),
                    new ControlPoint(1 ,  1, 0, 0.70710678),
                    new ControlPoint(0 ,  1, 0, 1),
                    new ControlPoint(-1,  1, 0, 0.70710678),
                    new ControlPoint(-1,  0, 0, 1),
                    new ControlPoint(-1, -1, 0, 0.70710678),
                    new ControlPoint(0 , -1, 0, 1),
                    new ControlPoint(1 , -1, 0, 0.70710678),
                    new ControlPoint(1 ,  0, 0, 1)
                ],
                [
                    new ControlPoint(1 ,  0, 1, 1),
                    new ControlPoint(1 ,  1, 1, 0.70710678),
                    new ControlPoint(0 ,  1, 1, 1),
                    new ControlPoint(-1,  1, 1, 0.70710678),
                    new ControlPoint(-1,  0, 1, 1),
                    new ControlPoint(-1, -1, 1, 0.70710678),
                    new ControlPoint(0 , -1, 1, 1),
                    new ControlPoint(1 , -1, 1, 0.70710678),
                    new ControlPoint(1 ,  0, 1, 1)
                ],
                [
                    new ControlPoint(1 ,  0, 2, 1),
                    new ControlPoint(1 ,  1, 2, 0.70710678),
                    new ControlPoint(0 ,  1, 2, 1),
                    new ControlPoint(-1,  1, 2, 0.70710678),
                    new ControlPoint(-1,  0, 2, 1),
                    new ControlPoint(-1, -1, 2, 0.70710678),
                    new ControlPoint(0 , -1, 2, 1),
                    new ControlPoint(1 , -1, 2, 0.70710678),
                    new ControlPoint(1 ,  0, 2, 1)
                ],
            ];
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
                var normal = SurfaceAnalyzer.EvaluateNormal(surface, u, v);

                var evalpt = SurfaceEvaluator.Evaluate(surface, u, v);
                var expected = new Vector3Double(evalpt.X, evalpt.Y, 0).normalized;
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(normal.magnitude, Is.EqualTo(expected.magnitude).Within(0.0001));
                    Assert.That(Math.Abs(expected.X), Is.EqualTo(Math.Abs(normal.X)).Within(0.0001));
                    Assert.That(Math.Abs(expected.Y), Is.EqualTo(Math.Abs(normal.Y)).Within(0.0001));
                    Assert.That(Math.Abs(expected.Z), Is.EqualTo(Math.Abs(normal.Z)).Within(0.0001));
                }


            }
        }
        [Test]
        public void SurfaceTangentTestA()
        {
            //Plane surface
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 1.0, 0.0, 1),
                    new ControlPoint(1.0, 1.0, 0.0, 1)
                ],
            ];
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
                var (tangentU, tangentV) = SurfaceAnalyzer.EvaluateTangents(surface, u, v);
                var expectedTangentU = new Vector3Double(0.0, 1.0, 0.0);
                var expectedTangentV = new Vector3Double(1.0, 0.0, 0.0);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(expectedTangentU.X, Is.EqualTo(tangentU.X).Within(0.000001));
                    Assert.That(expectedTangentU.Y, Is.EqualTo(tangentU.Y).Within(0.000001));
                    Assert.That(expectedTangentU.Z, Is.EqualTo(tangentU.Z).Within(0.000001));
                }

            }
        }

        [Test]
        public void SurfaceTangentTestB()
        {
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = [0, 0, 0.5, 1, 1];
            double[] knotsV = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(1 ,  0, 0, 1),
                    new ControlPoint(1 ,  1, 0, 0.70710678),
                    new ControlPoint(0 ,  1, 0, 1),
                    new ControlPoint(-1,  1, 0, 0.70710678),
                    new ControlPoint(-1,  0, 0, 1),
                    new ControlPoint(-1, -1, 0, 0.70710678),
                    new ControlPoint(0 , -1, 0, 1),
                    new ControlPoint(1 , -1, 0, 0.70710678),
                    new ControlPoint(1 ,  0, 0, 1)
                ],
                [
                    new ControlPoint(1 ,  0, 1, 1),
                    new ControlPoint(1 ,  1, 1, 0.70710678),
                    new ControlPoint(0 ,  1, 1, 1),
                    new ControlPoint(-1,  1, 1, 0.70710678),
                    new ControlPoint(-1,  0, 1, 1),
                    new ControlPoint(-1, -1, 1, 0.70710678),
                    new ControlPoint(0 , -1, 1, 1),
                    new ControlPoint(1 , -1, 1, 0.70710678),
                    new ControlPoint(1 ,  0, 1, 1)
                ],
                [
                    new ControlPoint(1 ,  0, 2, 1),
                    new ControlPoint(1 ,  1, 2, 0.70710678),
                    new ControlPoint(0 ,  1, 2, 1),
                    new ControlPoint(-1,  1, 2, 0.70710678),
                    new ControlPoint(-1,  0, 2, 1),
                    new ControlPoint(-1, -1, 2, 0.70710678),
                    new ControlPoint(0 , -1, 2, 1),
                    new ControlPoint(1 , -1, 2, 0.70710678),
                    new ControlPoint(1 ,  0, 2, 1)
                ],
            ];
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
                var (tangentU, tangentV) = SurfaceAnalyzer.EvaluateTangents(surface, u, v);
                var evalpt = SurfaceEvaluator.Evaluate(surface, u, v);
                var expectedTangentU = new Vector3Double(0, 0, 1);
                var expectedTangentV = new Vector3Double(-evalpt.Y, evalpt.X, 0).normalized;// tangent along the circular direction
                using (Assert.EnterMultipleScope())
                {
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
        }

        [Test]
        public void SurfaceCurvatureTestA()
        {
            var R = 3.0;
            //Cylinder surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = [0, 0, 0.5, 1, 1];
            double[] knotsV = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(R ,  0, 0, 1),
                    new ControlPoint(R ,  R, 0, 0.70710678),
                    new ControlPoint(0 ,  R, 0, 1),
                    new ControlPoint(-R,  R, 0, 0.70710678),
                    new ControlPoint(-R,  0, 0, 1),
                    new ControlPoint(-R, -R, 0, 0.70710678),
                    new ControlPoint(0 , -R, 0, 1),
                    new ControlPoint(R , -R, 0, 0.70710678),
                    new ControlPoint(R ,  0, 0, 1)
                ],
                [
                    new ControlPoint(R ,  0, 1, 1),
                    new ControlPoint(R ,  R, 1, 0.70710678),
                    new ControlPoint(0 ,  R, 1, 1),
                    new ControlPoint(-R,  R, 1, 0.70710678),
                    new ControlPoint(-R,  0, 1, 1),
                    new ControlPoint(-R, -R, 1, 0.70710678),
                    new ControlPoint(0 , -R, 1, 1),
                    new ControlPoint(R , -R, 1, 0.70710678),
                    new ControlPoint(R ,  0, 1, 1)
                ],
                [
                    new ControlPoint(R ,  0, 2, 1),
                    new ControlPoint(R ,  R, 2, 0.70710678),
                    new ControlPoint(0 ,  R, 2, 1),
                    new ControlPoint(-R,  R, 2, 0.70710678),
                    new ControlPoint(-R,  0, 2, 1),
                    new ControlPoint(-R, -R, 2, 0.70710678),
                    new ControlPoint(0 , -R, 2, 1),
                    new ControlPoint(R , -R, 2, 0.70710678),
                    new ControlPoint(R ,  0, 2, 1)
                ],
            ];
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
                (double k1, double k2) = SurfaceAnalyzer.EvaluatePrincipalCurvatures(surface, u, v);
                (double H, double K) = SurfaceAnalyzer.EvaluateMeanAndGaussianCurvatures(surface, u, v);
                using (Assert.EnterMultipleScope())
                {
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
        }

        [Test]
        public void SurfaceCurvatureTestB()
        {
            //Plane surface
            int degreeU = 2;
            int degreeV = 2;
            double[] knotsU = [0, 0, 0, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 1, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1),
                    new ControlPoint(2.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 1.0, 0.0, 1),
                    new ControlPoint(1.0, 1.0, 0.0, 1),
                    new ControlPoint(2.0, 1.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 2.0, 0.0, 1),
                    new ControlPoint(1.0, 2.0, 0.0, 1),
                    new ControlPoint(2.0, 2.0, 0.0, 1)
                ],
            ];
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
                (double k1, double k2) = SurfaceAnalyzer.EvaluatePrincipalCurvatures(surface, u, v);
                (double H, double K) = SurfaceAnalyzer.EvaluateMeanAndGaussianCurvatures(surface, u, v);
                using (Assert.EnterMultipleScope())
                {
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

        [Test]
        public void FindClosestPoint_OnPlanarSurface_ReturnsPerpendicularPoint()
        {
            // Create a planar surface at Z=0, spanning (0,0,0) to (10,10,0)
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Target point above the surface
            var target = new Vector3Double(5, 5, 3);

            // Find closest point using operator
            var (u, v, point, distance) = SurfaceAnalyzer.FindClosestPoint(surface, target);

            using (Assert.EnterMultipleScope())
            {
                // Should find point at (5,5,0) - directly below target
                Assert.That(point.X, Is.EqualTo(5.0).Within(1e-3), "X coordinate should be 5");
                Assert.That(point.Y, Is.EqualTo(5.0).Within(1e-3), "Y coordinate should be 5");
                Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-3), "Z coordinate should be 0");
                Assert.That(distance, Is.EqualTo(3.0).Within(1e-3), "Distance should be 3");
            }
        }

        [Test]
        public void FindClosestPoint_OnPlanarSurface_UsingInstanceMethod()
        {
            // Create a planar surface
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Target point above the surface
            var target = new Vector3Double(7, 3, 2);

            // Find closest point using instance method
            var (u, v, point, distance) = surface.FindClosestPoint(target);

            using (Assert.EnterMultipleScope())
            {
                // Should find point at (7,3,0)
                Assert.That(point.X, Is.EqualTo(7.0).Within(1e-3), "X coordinate should be 7");
                Assert.That(point.Y, Is.EqualTo(3.0).Within(1e-3), "Y coordinate should be 3");
                Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-3), "Z coordinate should be 0");
                Assert.That(distance, Is.EqualTo(2.0).Within(1e-3), "Distance should be 2");
            }
        }

        [Test]
        public void FindClosestPoint_OnCylindricalSurface_ReturnsRadialPoint()
        {
            // Create a cylinder
            double radius = 5.0;
            double height = 10.0;
            var cylinders = PrimitiveFactory.CreateCylinder(radius, height, true);
            var cylinder = cylinders[0]; // Side surface

            // Target point at the axis (center)
            var target = new Vector3Double(0, 0, 0);

            // Find closest point
            var (u, v, point, distance) = SurfaceAnalyzer.FindClosestPoint(cylinder, target);

            // Distance should be approximately radius
            Assert.That(distance, Is.EqualTo(radius).Within(0.1), "Distance should be approximately radius");

            // Point should be on the cylinder surface
            double radiusCheck = Math.Sqrt(point.X * point.X + point.Y * point.Y);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(radiusCheck, Is.EqualTo(radius).Within(1e-2), "Point should be on the cylindrical surface");
                Assert.That(Math.Abs(point.Z), Is.LessThan(height / 2 + 0.1), "Z should be within cylinder height");
            }
        }

        [Test]
        public void FindClosestPoint_OnSphericalSurface_ReturnsRadialPoint()
        {
            // Create a sphere
            double radius = 7.5;
            var sphere = PrimitiveFactory.CreateSphere(radius);

            // Target point at the origin (center)
            var target = new Vector3Double(0, 0, 0);

            // Find closest point
            var (u, v, point, distance) = SurfaceAnalyzer.FindClosestPoint(sphere, target);

            // Distance should be approximately radius
            Assert.That(distance, Is.EqualTo(radius).Within(0.1), "Distance should be approximately radius");

            // Point should be on the sphere surface
            double radiusCheck = Math.Sqrt(point.X * point.X + point.Y * point.Y + point.Z * point.Z);
            Assert.That(radiusCheck, Is.EqualTo(radius).Within(0.1), "Point should be on the spherical surface");
        }

        [Test]
        public void FindClosestPoint_WithInitialGuess_ConvergesFaster()
        {
            // Create a planar surface
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Target point
            var target = new Vector3Double(8, 6, 4);

            // Find closest point with good initial guess
            var (u, v, point, distance) = SurfaceAnalyzer.FindClosestPoint(surface, target, initialU: 0.6, initialV: 0.8);

            using (Assert.EnterMultipleScope())
            {
                // Should find point at (8,6,0)
                Assert.That(point.X, Is.EqualTo(8.0).Within(1e-3), "X coordinate should be 8");
                Assert.That(point.Y, Is.EqualTo(6.0).Within(1e-3), "Y coordinate should be 6");
                Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-3), "Z coordinate should be 0");
                Assert.That(distance, Is.EqualTo(4.0).Within(1e-3), "Distance should be 4");
            }
        }

        [Test]
        public void FindClosestPoint_WithInitialGuess_UsingInstanceMethod()
        {
            // Create a planar surface
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            var target = new Vector3Double(8, 6, 4);

            // Use instance method with initial guess
            var (u, v, point, distance) = surface.FindClosestPointWithInitialGuess(target, initialU: 0.6, initialV: 0.8);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(point.X, Is.EqualTo(8.0).Within(1e-3));
                Assert.That(point.Y, Is.EqualTo(6.0).Within(1e-3));
                Assert.That(distance, Is.EqualTo(4.0).Within(1e-3));
            }
        }

        [Test]
        public void FindClosestPoint_PointOnSurface_ReturnsZeroDistance()
        {
            // Create a planar surface
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Target point exactly on the surface
            var target = new Vector3Double(5, 5, 0);

            var (u, v, point, distance) = SurfaceAnalyzer.FindClosestPoint(surface, target);

            using (Assert.EnterMultipleScope())
            {
                // Distance should be essentially zero
                Assert.That(distance, Is.LessThan(1e-4), "Distance should be near zero");
                Assert.That(point.X, Is.EqualTo(5.0).Within(1e-3));
                Assert.That(point.Y, Is.EqualTo(5.0).Within(1e-3));
            }
        }
    }
}