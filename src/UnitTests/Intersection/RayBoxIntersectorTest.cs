using System;
using NurbsSharp.Core;
using NurbsSharp.Intersection;
using NUnit.Framework;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class RayBoxIntersectorTest
    {
        [Test]
        public void RayBoxIntersects_RayHitsBox()
        {
            // Ray pointing at box
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxIntersects_RayMissesBox()
        {
            // Ray pointing away from box
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(-1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.False);
        }

        [Test]
        public void RayBoxIntersects_RayOriginInsideBox()
        {
            // Ray starts inside the box
            var ray = new Ray(new Vector3Double(5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxIntersects_RayBehindBox()
        {
            // Box is behind the ray origin
            var ray = new Ray(new Vector3Double(15, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.False);
        }

        [Test]
        public void RayBoxIntersects_RayParallelToBoxFace()
        {
            // Ray parallel to X-axis, passing through box
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxIntersects_RayParallelMisses()
        {
            // Ray parallel to box but doesn't intersect
            var ray = new Ray(new Vector3Double(-5, 15, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.False);
        }

        [Test]
        public void RayBoxIntersects_WithParameters()
        {
            // Ray hits box, check tMin and tMax
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            bool intersects = RayBoxIntersector.Intersects(ray, box, out double tMin, out double tMax);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersects, Is.True);
                Assert.That(tMin, Is.EqualTo(5).Within(1e-10)); // Enters at x=0
                Assert.That(tMax, Is.EqualTo(15).Within(1e-10)); // Exits at x=10
            }
        }

        [Test]
        public void RayBoxIntersects_DiagonalRay()
        {
            // Ray at 45 degrees
            var ray = new Ray(new Vector3Double(-5, -5, 5), new Vector3Double(1, 1, 0).normalized);
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            bool intersects = RayBoxIntersector.Intersects(ray, box, out double tMin, out double tMax);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersects, Is.True);
                Assert.That(tMin, Is.GreaterThan(0));
                Assert.That(tMax, Is.GreaterThan(tMin));
            }
        }

        [Test]
        public void RayBoxIntersects_RayFromInside()
        {
            // Ray starts inside, tMin should be negative
            var ray = new Ray(new Vector3Double(5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            bool intersects = RayBoxIntersector.Intersects(ray, box, out double tMin, out double tMax);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersects, Is.True);
                Assert.That(tMin, Is.LessThan(0)); // Already inside
                Assert.That(tMax, Is.GreaterThan(0)); // Exit point
            }
        }

        [Test]
        public void RayBoxIntersects_EdgeCase_RayAlongEdge()
        {
            // Ray along the edge of the box
            var ray = new Ray(new Vector3Double(0, 0, 0), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxGetIntersectionPoint_Success()
        {
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            bool result = RayBoxIntersector.GetIntersectionPoint(ray, box, out Vector3Double point);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(result, Is.True);
                Assert.That(point.X, Is.EqualTo(0).Within(1e-10));
                Assert.That(point.Y, Is.EqualTo(5).Within(1e-10));
                Assert.That(point.Z, Is.EqualTo(5).Within(1e-10));
            }
        }

        [Test]
        public void RayBoxGetIntersectionPoint_FromInside()
        {
            // Ray starts inside, should return exit point
            var ray = new Ray(new Vector3Double(5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            bool result = RayBoxIntersector.GetIntersectionPoint(ray, box, out Vector3Double point);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(result, Is.True);
                Assert.That(point.X, Is.EqualTo(10).Within(1e-10));
                Assert.That(point.Y, Is.EqualTo(5).Within(1e-10));
                Assert.That(point.Z, Is.EqualTo(5).Within(1e-10));
            }
        }

        [Test]
        public void RayBoxGetIntersectionPoint_Miss()
        {
            var ray = new Ray(new Vector3Double(-5, 15, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            bool result = RayBoxIntersector.GetIntersectionPoint(ray, box, out Vector3Double point);

            Assert.That(result, Is.False);
        }

        [Test]
        public void RayBoxIntersectsSegment_WithinRange()
        {
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            // Max distance covers the intersection
            Assert.That(RayBoxIntersector.IntersectsSegment(ray, box, 20), Is.True);
        }

        [Test]
        public void RayBoxIntersectsSegment_OutOfRange()
        {
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            // Max distance too short to reach the box
            Assert.That(RayBoxIntersector.IntersectsSegment(ray, box, 3), Is.False);
        }

        [Test]
        public void RayBoxIntersectsSegment_ExactlyAtEdge()
        {
            var ray = new Ray(new Vector3Double(-5, 5, 5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            // Max distance exactly reaches the box entry
            Assert.That(RayBoxIntersector.IntersectsSegment(ray, box, 5), Is.True);
        }

        [Test]
        public void RayBoxIntersects_3DCase()
        {
            // Ray in full 3D space
            var ray = new Ray(
                new Vector3Double(-10, -10, -10),
                new Vector3Double(1, 1, 1),
                true // normalize
            );
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(5, 5, 5)
            );

            bool intersects = RayBoxIntersector.Intersects(ray, box, out double tMin, out double tMax);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersects, Is.True);
                Assert.That(tMin, Is.GreaterThan(0));
                Assert.That(tMax, Is.GreaterThan(tMin));
                //Intersect at (0,0,0)
                var intersectionPoint = ray.PointAt(tMin);
                Assert.That(intersectionPoint.X, Is.EqualTo(0).Within(1e-10));
                Assert.That(intersectionPoint.Y, Is.EqualTo(0).Within(1e-10));
                Assert.That(intersectionPoint.Z, Is.EqualTo(0).Within(1e-10));
                //Exit at (5,5,5)
                var exitPoint = ray.PointAt(tMax);
                Assert.That(exitPoint.X, Is.EqualTo(5).Within(1e-10));
                Assert.That(exitPoint.Y, Is.EqualTo(5).Within(1e-10));
                Assert.That(exitPoint.Z, Is.EqualTo(5).Within(1e-10));

            }
        }

        [Test]
        public void RayBoxIntersects_NegativeDirection()
        {
            // Ray with negative direction components
            var ray = new Ray(new Vector3Double(15, 15, 15), new Vector3Double(-1, -1, -1).normalized);
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(10, 10, 10));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxIntersects_SmallBox()
        {
            // Very small box
            var ray = new Ray(new Vector3Double(-1, 0.5, 0.5), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(1, 1, 1));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxIntersects_LargeBox()
        {
            // Very large box
            var ray = new Ray(new Vector3Double(-100, 50, 50), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(100, 100, 100));

            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }

        [Test]
        public void RayBoxIntersects_ZeroBox()
        {
            var ray = new Ray(new Vector3Double(-1, 0, 0), new Vector3Double(1, 0, 0));
            var box = new BoundingBox(new Vector3Double(0, 0, 0), new Vector3Double(0, 0, 0));
            Assert.That(RayBoxIntersector.Intersects(ray, box), Is.True);
        }
    }
}
