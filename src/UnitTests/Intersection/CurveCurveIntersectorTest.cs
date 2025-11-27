using System;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Intersection;
using NUnit.Framework;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class CurveCurveIntersectorTest
    {
        [Test]
        public void TwoIntersectingLines_FindsIntersection()
        {
            // Create two intersecting straight lines (degree 1)
            // Line 1: from (0,0,0) to (2,0,0) - horizontal
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 1, 1], 1);
            var curve1 = new NurbsCurve(1, knots1, cp1);

            // Line 2: from (1,-1,0) to (1,1,0) - vertical
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(1, -1, 0),
                new ControlPoint(1, 1, 0)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);

            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);//hit (1,0,0)

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(1));
                var intersection = intersections[0];
                Assert.That(intersection.Point1.X, Is.EqualTo(1.0).Within(CurveCurveIntersector.Tolerance));
                Assert.That(intersection.Point1.Y, Is.EqualTo(0.0).Within(CurveCurveIntersector.Tolerance));
                Assert.That(intersection.Point1.Z, Is.EqualTo(0.0).Within(CurveCurveIntersector.Tolerance));
                Assert.That(intersection.Distance, Is.LessThan(CurveCurveIntersector.Tolerance));
            }
        }

        [Test]
        public void ParallelLines_NoIntersection()
        {
            // Create two parallel horizontal lines
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 1, 1], 1);
            var curve1 = new NurbsCurve(1, knots1, cp1);

            var cp2 = new ControlPoint[]
            {
                new ControlPoint(0, 1, 0),
                new ControlPoint(2, 1, 0)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);

            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);

            Assert.That(intersections, Is.Empty);
        }

        [Test]
        public void DisjointBoundingBoxes_NoIntersection()
        {
            // Create two curves that are far apart
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 1, 1], 1);
            var curve1 = new NurbsCurve(1, knots1, cp1);

            var cp2 = new ControlPoint[]
            {
                new ControlPoint(10, 10, 0),
                new ControlPoint(11, 10, 0)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);

            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);

            Assert.That(intersections, Is.Empty);
        }

        [Test]
        public void QuadraticCurveIntersectingLine()
        {
            // Quadratic parabola-like curve
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(0.5, 1, 0),
                new ControlPoint(1, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var curve1 = new NurbsCurve(2, knots1, cp1); // (0,0) -> (0.5,0.5) -> (1,0)

            // Horizontal line at y=0.5
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(-0.5, 0.5, 0),
                new ControlPoint(1.5, 0.5, 0)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);

            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);// hit at (0.5,0.5,0)

            // Should find at least 1 intersection (algorithm may miss some on complex curves)
            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(1));
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Point1.X, Is.EqualTo(0.5).Within(CurveCurveIntersector.Tolerance));
                    Assert.That(intersection.Point1.Y, Is.EqualTo(0.5).Within(CurveCurveIntersector.Tolerance));
                    Assert.That(intersection.Point1.Z, Is.EqualTo(0.0).Within(CurveCurveIntersector.Tolerance));
                }
            }
        }

        [Test]
        public void Intersects_BooleanTest_True()
        {
            // Two intersecting lines
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 1, 1], 1);
            var curve1 = new NurbsCurve(1, knots1, cp1);//horizontal line y=0

            var cp2 = new ControlPoint[]
            {
                new ControlPoint(1, -1, 0),
                new ControlPoint(1, 1, 0)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);//vertical line x=1

            bool intersects = CurveCurveIntersector.Intersects(curve1, curve2);// hit at (1,0,0)

            Assert.That(intersects, Is.True);
        }

        [Test]
        public void Intersects_BooleanTest_False()
        {
            // Two parallel lines
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 1, 1], 1);
            var curve1 = new NurbsCurve(1, knots1, cp1);//horizontal line y=0

            var cp2 = new ControlPoint[]
            {
                new ControlPoint(0, 1, 0),
                new ControlPoint(2, 1, 0)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);//horizontal line y=1

            bool intersects = CurveCurveIntersector.Intersects(curve1, curve2);

            Assert.That(intersects, Is.False);
        }


        [Test]
        public void IdenticalCurves_FoundAsIntersecting()
        {
            // Two identical curves should be detected as intersecting
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 0),
                new ControlPoint(2, 0, 0)
            };
            var knots = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var curve1 = new NurbsCurve(2, knots, cp1);
            var curve2 = new NurbsCurve(2, knots, cp1);

            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);

            // Should find many intersections along the entire curve
            Assert.That(intersections, Has.Count.GreaterThanOrEqualTo(1));
        }

        [Test]
        public void ThreeDimensionalCurves_FindsIntersection()
        {
            // Create two 3D curves that intersect
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 0, 1)
            };
            var knots1 = new KnotVector([0, 0, 1, 1], 1);
            var curve1 = new NurbsCurve(1, knots1, cp1);// from (0,0,0) to (1,0,1)

            var cp2 = new ControlPoint[]
            {
                new ControlPoint(0.5, -1, 0.5),
                new ControlPoint(0.5, 1, 0.5)
            };
            var knots2 = new KnotVector([0, 0, 1, 1], 1);
            var curve2 = new NurbsCurve(1, knots2, cp2);// x=0.5, z=0.5 vertical line

            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);//hit at (0.5,0,0.5)

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(1));
                var intersection = intersections[0];
                Assert.That(intersection.Point1.X, Is.EqualTo(0.5).Within(CurveCurveIntersector.Tolerance));
                Assert.That(intersection.Point1.Y, Is.EqualTo(0.0).Within(CurveCurveIntersector.Tolerance));
                Assert.That(intersection.Point1.Z, Is.EqualTo(0.5).Within(CurveCurveIntersector.Tolerance));
                Assert.That(intersection.Distance, Is.LessThan(CurveCurveIntersector.Tolerance));
            }
        }

        [Test]
        public void P3CurveIntersectingLine()
        {
            // Cubic curve
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(0.33, 1, 0),
                new ControlPoint(0.66, -1, 0),
                new ControlPoint(1, 0, 0)
            };
            var knots1 = new KnotVector([0, 0, 0, 0, 1, 1, 1, 1], 3);
            var curve1 = new NurbsCurve(3, knots1, cp1); // Cubic curve from (0,0,0) to (1,0,0)
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(-1, 0, 0),
                new ControlPoint(0, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var knots2 = new KnotVector([0, 0, 0, 1, 1, 1], 2);
            var curve2 = new NurbsCurve(2, knots2, cp2);
            var intersections = CurveCurveIntersector.Intersect(curve1, curve2);
            // Should find at least 3 intersections (at endpoints and middle crossing)
            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(3));
                // Verify all intersections are near y=0
                foreach (var intersection in intersections)
                {
                    Assert.That(intersection.Point1.Y, Is.EqualTo(0.0).Within(CurveCurveIntersector.Tolerance));
                    Assert.That(intersection.Distance, Is.LessThan(CurveCurveIntersector.Tolerance));
                }
            }
        }
    }
}
