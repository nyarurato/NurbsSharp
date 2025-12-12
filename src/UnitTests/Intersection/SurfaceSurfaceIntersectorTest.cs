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

        /// <summary>
        /// Test that intersection curves have appropriate bounding box extent.
        /// Validates that the intersection algorithm produces complete curves without truncation.
        /// </summary>
        [Test]
        public void IntersectCurves_PlanePlane_BoundingBoxCoverage()
        {
            // Plane A: z = 0 (X-Y plane), extended from (0,0) to (5,5)
            var surfaceCpA = new ControlPoint[][]
            {
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 5, 0)
                ],
                [
                    new ControlPoint(5, 0, 0),
                    new ControlPoint(5, 5, 0)
                ]
            };
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, 1);
            var planeA = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCpA);

            // Plane B: x = 2.5 (parallel to YZ plane)
            var surfaceCpB = new ControlPoint[][]
            {
                [
                    new ControlPoint(2.5, 0, -2),
                    new ControlPoint(2.5, 0, 2)
                ],
                [
                    new ControlPoint(2.5, 5, -2),
                    new ControlPoint(2.5, 5, 2)
                ]
            };
            var planeB = new NurbsSurface(1, 1, knotsU, knotsV, surfaceCpB);

            var curves = SurfaceSurfaceIntersector.IntersectCurves(planeA, planeB, 
                SurfaceSurfaceIntersector.Tolerance, isoDivisions: 20, interpolate: true, degree: 1);

            Assert.That(curves, Is.Not.Empty, "No intersection curves found");

            var curve = curves[0];
            var bbox = curve.BoundingBox;

            // Expected: Line from (2.5, 0, 0) to (2.5, 5, 0)
            double expectedYSpan = 5.0;
            double actualYSpan = bbox.Max.Y - bbox.Min.Y;

            Console.WriteLine($"Plane-Plane Intersection:");
            Console.WriteLine($"  Expected Y span: {expectedYSpan:F3}");
            Console.WriteLine($"  Actual Y span: {actualYSpan:F3}");
            Console.WriteLine($"  Coverage: {actualYSpan / expectedYSpan * 100:F1}%");
            Console.WriteLine($"  BBox: X=[{bbox.Min.X:F3}, {bbox.Max.X:F3}], Y=[{bbox.Min.Y:F3}, {bbox.Max.Y:F3}], Z=[{bbox.Min.Z:F3}, {bbox.Max.Z:F3}]");
            Console.WriteLine($"  Curve has {curve.ControlPoints.Length} control points");

            // Should span most of Y direction (0 to 5)
            Assert.That(actualYSpan, Is.GreaterThan(expectedYSpan * 0.9),
                $"Y span too small: {actualYSpan:F3}, expected at least {expectedYSpan * 0.9:F3} (90% of {expectedYSpan:F3})");

            // X should be close to 2.5
            Assert.That(bbox.Min.X, Is.EqualTo(2.5).Within(0.1), $"Min X incorrect: {bbox.Min.X}");
            Assert.That(bbox.Max.X, Is.EqualTo(2.5).Within(0.1), $"Max X incorrect: {bbox.Max.X}");

            // Z should be close to 0
            Assert.That(Math.Abs(bbox.Min.Z), Is.LessThan(0.1), $"Min Z should be near 0: {bbox.Min.Z}");
            Assert.That(Math.Abs(bbox.Max.Z), Is.LessThan(0.1), $"Max Z should be near 0: {bbox.Max.Z}");
        }

        /// <summary>
        /// Test bounding box coverage for curved surface-surface intersection.
        /// </summary>
        [Test]
        public void IntersectCurves_CurvedSurfaces_BoundingBoxCoverage()
        {
            // Surface 1: Quadratic surface with curvature
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

            // Surface 2: Planar surface intersecting surface1
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

            var curves = SurfaceSurfaceIntersector.IntersectCurves(surface1, surface2,
                SurfaceSurfaceIntersector.Tolerance, isoDivisions: 20, interpolate: true, degree: 2);

            Assert.That(curves, Is.Not.Empty, "No intersection curves found");

            // Check bounding box of intersection curve
            var curve = curves[0];
            var bbox = curve.BoundingBox;

            // Surface 1 Y range is [0, 2], Surface 2 Y range is [-1, 3]
            // Intersection should span Y approximately [0, 2]
            double expectedMinY = 0.0;
            double expectedMaxY = 2.0;
            double expectedYSpan = expectedMaxY - expectedMinY;
            double actualYSpan = bbox.Max.Y - bbox.Min.Y;

            Console.WriteLine($"\nCurved Surface-Surface Intersection:");
            Console.WriteLine($"  Surface1 BBox: {surface1.BoundingBox}");
            Console.WriteLine($"  Surface2 BBox: {surface2.BoundingBox}");
            Console.WriteLine($"  Expected Y span: ~{expectedYSpan:F3} (from {expectedMinY:F1} to {expectedMaxY:F1})");
            Console.WriteLine($"  Actual Y span: {actualYSpan:F3} (from {bbox.Min.Y:F3} to {bbox.Max.Y:F3})");
            Console.WriteLine($"  Coverage: {actualYSpan / expectedYSpan * 100:F1}%");
            Console.WriteLine($"  Full BBox: X=[{bbox.Min.X:F3}, {bbox.Max.X:F3}], Y=[{bbox.Min.Y:F3}, {bbox.Max.Y:F3}], Z=[{bbox.Min.Z:F3}, {bbox.Max.Z:F3}]");
            Console.WriteLine($"  Curve has {curve.ControlPoints.Length} control points");

            // Should span at least 60% of expected Y range (relaxed due to surface boundaries)
            Assert.That(actualYSpan, Is.GreaterThan(expectedYSpan * 0.8),
                $"Y span too small: {actualYSpan:F3}, expected at least {expectedYSpan * 0.8:F3} (80% of {expectedYSpan:F3})");

            // Y min should be close to 0 (or slightly above)
            Assert.That(bbox.Min.Y, Is.GreaterThanOrEqualTo(expectedMinY - 0.2),
                $"Min Y out of range: {bbox.Min.Y:F3}, expected >= {expectedMinY - 0.2:F3}");

            // Y max should be close to 2 (or slightly below)
            Assert.That(bbox.Max.Y, Is.LessThanOrEqualTo(expectedMaxY + 0.2),
                $"Max Y out of range: {bbox.Max.Y:F3}, expected <= {expectedMaxY + 0.2:F3}");
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
