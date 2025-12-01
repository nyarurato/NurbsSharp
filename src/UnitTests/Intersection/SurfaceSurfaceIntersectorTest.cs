using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Intersection;
using NurbsSharp.Evaluation;
using NUnit.Framework;
using System.IO;
using NurbsSharp.IO.IGES;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class SurfaceSurfaceIntersectorTest
    {
        [Test]
        public void PlanePlanePerpendicular_FindsIntersectionLine()
        {
            // Plane A: z = 0 (X-Y plane)
            var surfaceCpA = new ControlPoint[][]
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
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var planeA = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCpA);

            // Plane B: x = 0 (Y-Z plane)
            var surfaceCpB = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 0, 1)
                ],
                [
                    new ControlPoint(0, 1, 0),
                    new ControlPoint(0, 1, 1)
                ]
            };
            var planeB = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCpB);

            var intersections = SurfaceSurfaceIntersector.Intersect(planeA, planeB, SurfaceSurfaceIntersector.Tolerance, isoDivisions: 10);

            Assert.That(intersections, Is.Not.Empty);

            // There should be an intersection somewhere along y in [0,1], e.g., y=0.5
            bool foundNearMid = false;
            foreach (var inter in intersections)
            {
                var p = inter.PointA; // point on plane A
                if (Math.Abs(p.X - 0.0) < 1e-6 && Math.Abs(p.Z - 0.0) < 1e-6 && p.Y > 0.0 && p.Y < 1.0)
                {
                    foundNearMid = true;
                    Assert.That(inter.Distance, Is.LessThan(1e-6));
                }
            }

            Assert.That(foundNearMid, Is.True);
        }

        [Test]
        public void p2surf_p1surf_intersect()
        {
            var degree_s1_u = 2;
            var degree_s1_v = 2;
            var cp_s1 = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0),
                    new ControlPoint(0, 2, 0)
                ],
                [
                    new ControlPoint(1, 0, 0),
                    new ControlPoint(1, 1, 1),
                    new ControlPoint(1, 2, 0)
                ],
                [
                    new ControlPoint(2, 0, 0),
                    new ControlPoint(2, 1, 0),
                    new ControlPoint(2, 2, 0)
                ]
            };
            var knots_s1_u = KnotVector.GetClampedKnot(degree_s1_u, cp_s1.Length);
            var knots_s1_v = KnotVector.GetClampedKnot(degree_s1_v, cp_s1[0].Length);
            var surface1 = new NurbsSurface(degree_s1_u, degree_s1_v, knots_s1_u, knots_s1_v, cp_s1);

            var degree_s2_u = 1;
            var degree_s2_v = 1;
            var cp_s2 = new ControlPoint[][]
            {
                [
                    new ControlPoint(0.5, -1, 0.5),
                    new ControlPoint(0.5, 3, 0.5)
                ],
                [
                    new ControlPoint(1.5, -1, -0.5),
                    new ControlPoint(1.5, 3, -0.5)
                ]
            };
            var knots_s2_u = KnotVector.GetClampedKnot(degree_s2_u, cp_s2.Length);
            var knots_s2_v = KnotVector.GetClampedKnot(degree_s2_v, cp_s2[0].Length);
            var surface2 = new NurbsSurface(degree_s2_u, degree_s2_v, knots_s2_u, knots_s2_v, cp_s2);

            //TestOutIGES([surface1, surface2], "p2surf_p1surf_intersect.igs");

            var intersections = SurfaceSurfaceIntersector.Intersect(surface1, surface2, SurfaceSurfaceIntersector.Tolerance, isoDivisions: 20);
            Assert.That(intersections, Is.Not.Empty);
        }

        [Test]
        public void PlanePlanePerpendicular_ReturnsCurve()
        {
            // Plane A: z = 0 (X-Y plane)
            var surfaceCpA = new ControlPoint[][]
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
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var planeA = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCpA);

            // Plane B: x = 0 (Y-Z plane)
            var surfaceCpB = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 0, 1)
                ],
                [
                    new ControlPoint(0, 1, 0),
                    new ControlPoint(0, 1, 1)
                ]
            };
            var planeB = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCpB);

            var curves = SurfaceSurfaceIntersector.IntersectCurves(planeA, planeB, SurfaceSurfaceIntersector.Tolerance, isoDivisions: 10, interpolate: true, degree: 1);

            Assert.That(curves, Is.Not.Empty);
            var c = curves[0];
            // Evaluate at parameter roughly halfway
            double tu = c.KnotVector.Knots[c.Degree] + (c.KnotVector.Knots[c.KnotVector.Length - c.Degree - 1] - c.KnotVector.Knots[c.Degree]) * 0.5;
            var p = CurveEvaluator.Evaluate(c, tu);

            // Should lie on intersection line x=0, z=0
            Assert.That(Math.Abs(p.X - 0.0), Is.LessThan(1e-6));
            Assert.That(Math.Abs(p.Z - 0.0), Is.LessThan(1e-6));
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
