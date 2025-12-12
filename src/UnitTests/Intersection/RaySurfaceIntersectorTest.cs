using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Intersection;
using NurbsSharp.Generation;
using System;
using System.Linq;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class RaySurfaceIntersectorTest
    {
        [Test]
        public void RayIntersectsFast_PlanarSurface_ReturnsCorrectPoint()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            var ray = new Ray(new Vector3Double(5, 5, 3), new Vector3Double(0, 0, -1));

            Assert.That(RaySurfaceIntersector.IntersectsFast(ray, surface, out var hit, numPointsU: 10, numPointsV: 10), Is.True);
            Assert.That(hit.Point.X, Is.EqualTo(5.0).Within(1e-3));
            Assert.That(hit.Point.Y, Is.EqualTo(5.0).Within(1e-3));
            Assert.That(hit.Point.Z, Is.EqualTo(0.0).Within(1e-3));
            Assert.That(hit.T, Is.EqualTo(3.0).Within(1e-3));
        }

        [Test]
        public void RayMissesFast_Surface_ReturnsFalse()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            var ray = new Ray(new Vector3Double(20, 20, 10), new Vector3Double(0, 0, -1));

            Assert.That(RaySurfaceIntersector.IntersectsFast(ray, surface, out var _), Is.False);
        }

        [Test]
        public void RayIntersectsFast_CylindricalSurface_ReturnsSideHit()
        {
            double radius = 5.0;
            double height = 10.0;
            var cylinders = PrimitiveFactory.CreateCylinder(radius, height, true);
            var cylinder = cylinders[0];

            // Ray from inside axis toward +X should hit side at x=radius
            var ray = new Ray(new Vector3Double(0, 0, 0), new Vector3Double(1, 0, 0));

            Assert.That(RaySurfaceIntersector.IntersectsFast(ray, cylinder, out var hit, numPointsU: 30, numPointsV: 60), Is.True);
            Assert.That(Math.Abs(hit.Point.X - radius), Is.LessThan(0.2), "X should be near radius");
            Assert.That(Math.Abs(hit.T - radius), Is.LessThan(0.2));
        }

        [Test]
        public void RayIntersectsFast_Sphere_ReturnsTwoHits_ForIntersectAll()
        {
            double radius = 7.5;
            var sphere = PrimitiveFactory.CreateSphere(radius);

            // Ray along -X through sphere center from outside
            var ray = new Ray(new Vector3Double(radius * 2.0, 0, 0), new Vector3Double(-1, 0, 0));

            var hits = RaySurfaceIntersector.IntersectAllFast(ray, sphere, numPointsU: 36, numPointsV: 72);
            // Should get two intersections (front and back)
            Assert.That(hits.Count, Is.GreaterThanOrEqualTo(2));
            // Closest hit should have t approx radius
            var first = hits.OrderBy(h => h.T).First();
            Assert.That(first.T, Is.EqualTo(radius).Within(0.5));
        }

        [Test]
        public void RayIntersectsRobust_PlanarSurface_ReturnsAccuratePoint()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            var ray = new Ray(new Vector3Double(5, 5, 3), new Vector3Double(0, 0, -1));

            Assert.That(RaySurfaceIntersector.IntersectsRobust(ray, surface, out var hit, tolerance: 1e-6), Is.True);
            Assert.That(hit.Point.X, Is.EqualTo(5.0).Within(1e-5), "X coordinate should be highly accurate");
            Assert.That(hit.Point.Y, Is.EqualTo(5.0).Within(1e-5), "Y coordinate should be highly accurate");
            Assert.That(hit.Point.Z, Is.EqualTo(0.0).Within(1e-5), "Z coordinate should be highly accurate");
            Assert.That(hit.T, Is.EqualTo(3.0).Within(1e-5), "Ray parameter should be highly accurate");
        }

        [Test]
        public void RayIntersectsRobust_Sphere_ReturnsTwoAccurateHits()
        {
            double radius = 7.5;
            var sphere = PrimitiveFactory.CreateSphere(radius);

            // Ray along -X through sphere center from outside
            var ray = new Ray(new Vector3Double(radius * 2.0, 0, 0), new Vector3Double(-1, 0, 0));

            var hits = RaySurfaceIntersector.IntersectAllRobust(ray, sphere, tolerance: 1e-6, tessellationResolution: 30);
            
            Assert.That(hits.Count, Is.GreaterThanOrEqualTo(2), "Should find at least two intersections through sphere");
            
            // First hit (entry point) should be at t ≈ radius
            var first = hits.OrderBy(h => h.T).First();
            Assert.That(first.T, Is.EqualTo(radius).Within(0.1), "Entry point should be accurate");
            
            // Last hit (exit point) should be at t ≈ 3*radius
            var last = hits.OrderBy(h => h.T).Last();
            Assert.That(last.T, Is.EqualTo(radius * 3.0).Within(0.1), "Exit point should be accurate");
        }

        [Test]
        public void CompareAccuracy_FastVsRobust_RobustIsMorePrecise()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            var ray = new Ray(new Vector3Double(3.7, 6.2, 5.5), new Vector3Double(0, 0, -1));

            // Fast version
            bool hitFast = RaySurfaceIntersector.IntersectsFast(ray, surface, out var hitDataFast, numPointsU: 20, numPointsV: 20);
            
            // Robust version
            bool hitRobust = RaySurfaceIntersector.IntersectsRobust(ray, surface, out var hitDataRobust, tolerance: 1e-6);

            Assert.That(hitFast, Is.True, "Fast should find intersection");
            Assert.That(hitRobust, Is.True, "Robust should find intersection");

            // Robust should be more accurate (Z should be exactly 0 for this planar surface)
            double fastError = Math.Abs(hitDataFast.Point.Z);
            double robustError = Math.Abs(hitDataRobust.Point.Z);
            
            Assert.That(robustError, Is.LessThan(1e-5), "Robust version should have very small error");
            // Note: Fast version error depends on tessellation, so we just check it's reasonable
            Assert.That(fastError, Is.LessThan(0.1), "Fast version should have acceptable error");
        }
    }
}
