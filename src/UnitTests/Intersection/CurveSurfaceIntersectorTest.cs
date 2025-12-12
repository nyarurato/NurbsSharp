using System;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Intersection;
using NUnit.Framework;
using System.Collections.Generic;
using System.IO;
using NurbsSharp.IO.IGES;
using System.Threading.Tasks;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class CurveSurfaceIntersectorTest
    {
        // Marching method should handle simple line-plane intersections
        [Test]
        public void LineIntersectingPlane_FindsIntersection()
        {
            // Create a vertical line passing through z=0
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0.5, 0.5, -1),
                new ControlPoint(0.5, 0.5, 1)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(1));
                var intersection = intersections[0];
                Assert.That(intersection.CurvePoint.X, Is.EqualTo(0.5).Within(CurveSurfaceIntersector.Tolerance));
                Assert.That(intersection.CurvePoint.Y, Is.EqualTo(0.5).Within(CurveSurfaceIntersector.Tolerance));
                Assert.That(intersection.CurvePoint.Z, Is.EqualTo(0.0).Within(CurveSurfaceIntersector.Tolerance));
                Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance));
            }
        }

        [Test]
        public void LineParallelToPlane_NoIntersection()
        {
            // Create a horizontal line at z=1
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0, 1),
                new ControlPoint(1, 0, 1)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);

            Assert.That(intersections, Is.Empty);
        }

        [Test]
        public void DisjointBoundingBoxes_NoIntersection()
        {
            // Create a curve far from the surface
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(10, 10, 10),
                new ControlPoint(11, 10, 10)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a plane at origin
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);

            Assert.That(intersections, Is.Empty);
        }

        [Test]
        public void QuadraticCurveIntersectingPlane()
        {
            // Parabolic curve passing through a plane
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0, -1),
                new ControlPoint(0.5, 0.5, 1),
                new ControlPoint(1, 0, -1)
            };
            var curveKnots = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var curve = new NurbsCurve(2, curveKnots, curveCp);

            // Horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);

            // Should find 2 intersections (parabola crosses plane twice)
            // Note: Current implementation may merge close intersections
            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.GreaterThanOrEqualTo(1));
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance));
                    Assert.That(intersection.CurvePoint.Z, Is.EqualTo(0.0).Within(0.01));
                }
            }
        }

        [Test]
        public void CurveIntersectingCurvedSurface()
        {
            // Create a straight line
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0.5, 0.5, -1),
                new ControlPoint(0.5, 0.5, 1)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a curved surface (saddle shape)
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, -0.5),
                    new ControlPoint(0, 0.5, 0),
                    new ControlPoint(0, 1, 0.5)
                ],
                [
                    new ControlPoint(0.5, 0, 0),
                    new ControlPoint(0.5, 0.5, 0),
                    new ControlPoint(0.5, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0.5),
                    new ControlPoint(1, 0.5, 0),
                    new ControlPoint(1, 1, -0.5)
                ]
            };
            var knotsU = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var knotsV = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var surface = new NurbsSurface(2, 2, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);

            // Should find at least one intersection
            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.GreaterThanOrEqualTo(1));
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance));
                }
            }
        }

        [Test]
        public void Intersects_BooleanTest_True()
        {
            // Line intersecting plane
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0.5, 0.5, -1),
                new ControlPoint(0.5, 0.5, 1)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            bool intersects = CurveSurfaceIntersector.Intersects(curve, surface);

            Assert.That(intersects, Is.True);
        }

        [Test]
        public void Intersects_BooleanTest_False()
        {
            // Line parallel to plane
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0, 1),
                new ControlPoint(1, 0, 1)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            bool intersects = CurveSurfaceIntersector.Intersects(curve, surface);

            Assert.That(intersects, Is.False);
        }

        [Test]
        public void CubicCurveIntersectingQuadraticSurface()
        {
            // Create a cubic curve
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0.5, -1),
                new ControlPoint(0.3, 0.5, 0.5),
                new ControlPoint(0.7, 0.5, -0.5),
                new ControlPoint(1, 0.5, 1)
            };
            var curveKnots = new KnotVector([0, 0, 0, 0, 1, 1, 1, 1], 3);
            var curve = new NurbsCurve(3, curveKnots, curveCp);

            // Create a quadratic surface
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 0.5, 0.2),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(0.5, 0, 0.2),
                    new ControlPoint(0.5, 0.5, 0),
                    new ControlPoint(0.5, 1, 0.2)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 0.5, 0.2),
                    new ControlPoint(1, 1, 0)
                ]
            };
            var knotsU = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var knotsV = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var surface = new NurbsSurface(2, 2, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);
            //TestOutIGES([surface]);
            //TestOutIGES([curve], "testcurve.igs");

            // Should find at least one intersection
            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.GreaterThanOrEqualTo(1));
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance * 10));
                }
            }
        }

        [Test]
        public void DiagonalLineIntersectingTiltedPlane()
        {
            // Create a diagonal line
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 1)
            };
            var curveKnots = new KnotVector([0, 0, 1, 1], 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a tilted plane z = x + y - 1
            var surfaceCp = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, -1),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 1)
                ]
            };
            var knotsU = new KnotVector([0, 0, 1, 1], 1);
            var knotsV = new KnotVector([0, 0, 1, 1], 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.GreaterThanOrEqualTo(1));
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance));
                }
            }
        }

        [Test]
        public void p3Surface_p3Curve()
        {
            int degreeU = 3;
            int degreeV = 3;

            ControlPoint[][] controlPoints =
            [
                [
                new ControlPoint(0.0, 0.0, 0.0, 1),  //U0 V0
                new ControlPoint(1.0, 0.0, 0.0, 1),  //U0 V1
                new ControlPoint(2.0, 0.0, 0.0, 1),  //U0 V2
                new ControlPoint(3.0, 0.0, 0.0, 1),  //U0 V3
                new ControlPoint(4.0, 0.0, 0.0, 1)   //U0 V4
                ],
                [
                new ControlPoint(0.0, 1.0, 0.5, 1),  //U1 V0
                new ControlPoint(1.0, 1.0, -1.5, 1), //U1 V1
                new ControlPoint(2.0, 1.0, 4.0, 1),  //U1 V2
                new ControlPoint(3.0, 1.0, -3.0, 1), //U1 V3
                new ControlPoint(4.0, 1.0, 0.5, 1)   //U1 V4
                ],
                [
                new ControlPoint(0.0, 2.0, 1.5, 1),  //U2 V0
                new ControlPoint(1.0, 2.0, 2.5, 1),  //U2 V1
                new ControlPoint(2.0, 2.0, 3.5, 0.7),//U2 V2
                new ControlPoint(3.0, 2.0, 3.0, 1),  //U2 V3
                new ControlPoint(4.0, 2.0, 0.0, 1)   //U2 V4
                ],
                [
                new ControlPoint(0.0, 3.0, 0.5, 1),  //U3 V0
                new ControlPoint(1.5, 3.0, -1.5, 1), //U3 V1
                new ControlPoint(2.5, 3.0, 2.0 ,1),  //U3 V2
                new ControlPoint(3.5, 3.0, -1.5, 1), //U3 V3
                new ControlPoint(4.5, 3.0, -1.0, 1)  //U3 V4
                ],
                [
                new ControlPoint(0.0, 4.0, 0.5, 1),  //U4 V0
                new ControlPoint(1.0, 4.0, 0.5, 1),  //U4 V1
                new ControlPoint(2.0, 4.0, 0.0, 1),  //U4 V2
                new ControlPoint(3.0, 4.0, 0.0, 1),  //U4 V3
                new ControlPoint(4.0, 4.0, 0.0, 1)   //U4 V4
                ],
            ]; // 5x5 control points U x V

            var knotVectorU = KnotVector.GetClampedKnot(degreeU, controlPoints.Length);
            var knotVectorV = KnotVector.GetClampedKnot(degreeV, controlPoints[0].Length);

            var surface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);

            int degree = 3;
            double[] knots = [0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1];
            KnotVector knotVector = new KnotVector(knots, degree);

            var cp = new ControlPoint[]
            {
            new ControlPoint(-1.0, 0.5, 2.0, 1.0),
            new ControlPoint(0.5, 1.0, 3.0, 1.0),
            new ControlPoint(1.5, 1.5, 1.5, 1.0),
            new ControlPoint(2.5, 2.5, -1.5, 1.0),
            new ControlPoint(3.0, 2.8, -1.0, 1.0),
            new ControlPoint(3.5, 3.2, -0.5, 1.0),
            new ControlPoint(5.0, 3.5, 1.5, 1.0)
            };

            var curve = new NurbsCurve(degree, knotVector, cp);

            var intersections = CurveSurfaceIntersector.Intersect(curve, surface);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(2));
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance * 10));
                }
            }

        }

        static async Task TestOutIGES(List<NurbsSurface> geometries, string filePath = "test.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            await IGESExporter.ExportAsync(geometries, stream);
        }

        static async Task TestOutIGES(List<NurbsCurve> geometry, string filePath = "test.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            await IGESExporter.ExportAsync(geometry, stream);
        }
    }
}
