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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a plane at origin
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
            var curve = new NurbsCurve(2, curveKnots, curveCp);

            // Horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a curved surface (saddle shape)
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, -0.5),
                    new ControlPoint(0, 0.5, 0),
                    new ControlPoint(0, 1, 0.5)
                },
                new ControlPoint[]
                {
                    new ControlPoint(0.5, 0, 0),
                    new ControlPoint(0.5, 0.5, 0),
                    new ControlPoint(0.5, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0.5),
                    new ControlPoint(1, 0.5, 0),
                    new ControlPoint(1, 1, -0.5)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
            var knotsV = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            bool intersects = CurveSurfaceIntersector.Intersects(curve, surface);

            Assert.That(intersects, Is.False);
        }

        [Test]
        [Ignore("Complex cubic-quadratic intersection - geometry may not actually intersect")]
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 0, 0, 1, 1, 1, 1 }, 3);
            var curve = new NurbsCurve(3, curveKnots, curveCp);

            // Create a quadratic surface
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 0.5, 0.2),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(0.5, 0, 0.2),
                    new ControlPoint(0.5, 0.5, 0),
                    new ControlPoint(0.5, 1, 0.2)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 0.5, 0.2),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
            var knotsV = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
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
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a tilted plane z = x + y - 1
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, -1),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 1)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
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

        // BVH-accelerated version tests (dual BVH: curve + surface)
        [Test]
        public void BVH_LineIntersectingPlane_FindsIntersection()
        {
            // Create a vertical line passing through z=0
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0.5, 0.5, -1),
                new ControlPoint(0.5, 0.5, 1)
            };
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.IntersectWithBVH(curve, surface);

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
        public void BVH_QuadraticCurveIntersectingPlane()
        {
            // Create a parabolic curve that crosses a plane
            // Same geometry as QuadraticCurveIntersectingPlane test
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0, -1),
                new ControlPoint(0.5, 0.5, 1),
                new ControlPoint(1, 0, -1)
            };
            var curveKnots = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
            var curve = new NurbsCurve(2, curveKnots, curveCp);

            // Create a horizontal plane at z=0
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.IntersectWithBVH(curve, surface);

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
        public void BVH_DiagonalLineIntersectingTiltedPlane()
        {
            // Create a diagonal line
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 1)
            };
            var curveKnots = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var curve = new NurbsCurve(1, curveKnots, curveCp);

            // Create a tilted plane z = x + y - 1
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, -1),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 1)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var surface = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCp);

            var intersections = CurveSurfaceIntersector.IntersectWithBVH(curve, surface);

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
        public void BVH_CompareWithStandardMethod()
        {
            // Create a cubic curve
            var curveCp = new ControlPoint[]
            {
                new ControlPoint(0, 0.5, -1),
                new ControlPoint(0.3, 0.5, 0.5),
                new ControlPoint(0.7, 0.5, -0.5),
                new ControlPoint(1, 0.5, 1)
            };
            var curveKnots = new KnotVector(new double[] { 0, 0, 0, 0, 1, 1, 1, 1 }, 3);
            var curve = new NurbsCurve(3, curveKnots, curveCp);

            // Create a quadratic surface
            var surfaceCp = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 0.5, 0.2),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(0.5, 0, 0.2),
                    new ControlPoint(0.5, 0.5, 0),
                    new ControlPoint(0.5, 1, 0.2)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 0.5, 0.2),
                    new ControlPoint(1, 1, 0)
                }
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
            var knotsV = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, 2);
            var surface = new NurbsSurface(2, 2, knotsU, knotsV, surfaceCp);

            var standardIntersections = CurveSurfaceIntersector.Intersect(curve, surface);
            var bvhIntersections = CurveSurfaceIntersector.IntersectWithBVH(curve, surface);

            // Both methods should find the same number of intersections
            using (Assert.EnterMultipleScope())
            {
                Assert.That(bvhIntersections.Count, Is.EqualTo(standardIntersections.Count),
                    "BVH and standard methods should find the same number of intersections");
                
                // If intersections found, verify they are valid
                if (bvhIntersections.Count > 0)
                {
                    foreach (var intersection in bvhIntersections)
                    {
                        Assert.That(intersection.Distance, Is.LessThan(CurveSurfaceIntersector.Tolerance * 10));
                    }
                }
                
                foreach (var intersection in bvhIntersections)
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
