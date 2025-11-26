using System;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Intersection;
using NUnit.Framework;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class RayMeshIntersectorTest
    {
        [Test]
        public void RayTriangleIntersects_Hit()
        {
            // Triangle in XY plane
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            // Ray pointing at the triangle center from above
            var ray = new Ray(new Vector3Double(0.25, 0.25, 1), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.T, Is.EqualTo(1.0).Within(1e-10));
                Assert.That(intersection.Point.X, Is.EqualTo(0.25).Within(1e-10));
                Assert.That(intersection.Point.Y, Is.EqualTo(0.25).Within(1e-10));
                Assert.That(intersection.Point.Z, Is.EqualTo(0).Within(1e-10));
            }
        }

        [Test]
        public void RayTriangleIntersects_Miss()
        {
            // Triangle in XY plane
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            // Ray pointing away from the triangle
            var ray = new Ray(new Vector3Double(0.25, 0.25, 1), new Vector3Double(0, 0, 1));

            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out _);

            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayTriangleIntersects_RayParallelToTriangle()
        {
            // Triangle in XY plane
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            // Ray parallel to triangle plane
            var ray = new Ray(new Vector3Double(0, 0, 1), new Vector3Double(1, 0, 0));

            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out _);

            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayTriangleIntersects_RayBehindTriangle()
        {
            // Triangle in XY plane
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            // Ray origin behind the triangle
            var ray = new Ray(new Vector3Double(0.25, 0.25, -1), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out _);

            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayTriangleIntersects_EdgeCase_RayHitsVertex()
        {
            // Triangle in XY plane
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            // Ray pointing at a vertex
            var ray = new Ray(new Vector3Double(0, 0, 1), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.Point.X, Is.EqualTo(0).Within(1e-10));
                Assert.That(intersection.Point.Y, Is.EqualTo(0).Within(1e-10));
                Assert.That(intersection.Point.Z, Is.EqualTo(0).Within(1e-10));
            }
        }

        [Test]
        public void RayTriangleIntersects_BarycentricCoordinates()
        {
            // Triangle vertices
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            // Ray hitting at (0.5, 0.3, 0) - should give u=0.5, v=0.3
            var ray = new Ray(new Vector3Double(0.5, 0.3, 1), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.U, Is.EqualTo(0.5).Within(1e-10));
                Assert.That(intersection.V, Is.EqualTo(0.3).Within(1e-10));
                // Verify: (1-u-v)*v0 + u*v1 + v*v2 = (0.2)*v0 + 0.5*v1 + 0.3*v2 = (0.5, 0.3, 0)
                Assert.That(intersection.U + intersection.V, Is.LessThanOrEqualTo(1.0));// u + v should be <= 1
            }
        }

        [Test]
        public void RayTriangleIntersects_EdgeCase_RayFromTriangle()
        {
            var v0 = new Vector3Double(0, 0, 0);
            var v1 = new Vector3Double(1, 0, 0);
            var v2 = new Vector3Double(0, 1, 0);

            var ray = new Ray(new Vector3Double(0.25, 0.25, 0), new Vector3Double(0, 0, -1));
            bool hit = RayMeshIntersector.IntersectsTriangle(ray, v0, v1, v2, out _);
            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayMeshIntersects_SimpleMesh_Hit()
        {
            // Create a simple mesh: single triangle
            var vertices = new[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0)
            };
            var indexes = new[] { 0, 1, 2 };
            var mesh = new Mesh(vertices, indexes);

            // Ray pointing at the triangle
            var ray = new Ray(new Vector3Double(0.25, 0.25, 1), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.Intersects(ray, mesh, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.TriangleIndex, Is.EqualTo(0));
                Assert.That(intersection.T, Is.EqualTo(1.0).Within(1e-10));
            }
        }

        [Test]
        public void RayMeshIntersects_SimpleMesh_Miss()
        {
            // Create a simple mesh: single triangle
            var vertices = new[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0)
            };
            var indexes = new[] { 0, 1, 2 };
            var mesh = new Mesh(vertices, indexes);

            // Ray pointing away
            var ray = new Ray(new Vector3Double(0.25, 0.25, 1), new Vector3Double(0, 0, 1));

            bool hit = RayMeshIntersector.Intersects(ray, mesh, out _);

            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayMeshIntersects_MultiTriangle_ClosestHit()
        {
            // Create a mesh with two triangles at different distances
            var vertices = new[]
            {
                // Triangle 1 (far) at z=0
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0),
                // Triangle 2 (near) at z=5
                new Vector3Double(0, 0, 5),
                new Vector3Double(1, 0, 5),
                new Vector3Double(0, 1, 5)
            };
            var indexes = new[] { 0, 1, 2, 3, 4, 5 };
            var mesh = new Mesh(vertices, indexes);

            // Ray from above, should hit the nearer triangle first
            var ray = new Ray(new Vector3Double(0.25, 0.25, 10), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.Intersects(ray, mesh, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.TriangleIndex, Is.EqualTo(1)); // Second triangle (index 1)
                Assert.That(intersection.T, Is.EqualTo(5.0).Within(1e-10)); // Distance to z=5
                Assert.That(intersection.Point.Z, Is.EqualTo(5.0).Within(1e-10));
            }
        }

        [Test]
        public void RayMeshIntersects_TripleTriangle_ClosestHit()
        {
            // Create a mesh with two triangles at different distances
            var vertices = new[]
            {
                // Triangle 1 (far) at z=0
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0),
                // Triangle 2 (near) at z=5
                new Vector3Double(0, 0, 5),
                new Vector3Double(1, 0, 5),
                new Vector3Double(0, 1, 5),
                // Triangle 3 (middle) at z=2
                new Vector3Double(0, 0, 2),
                new Vector3Double(1, 0, 2),
                new Vector3Double(0, 1, 2)
            };
            var indexes = new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
            var mesh = new Mesh(vertices, indexes);

            // Ray from above, should hit the nearer triangle first
            var ray = new Ray(new Vector3Double(0.25, 0.25, 10), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.Intersects(ray, mesh, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.TriangleIndex, Is.EqualTo(1)); // Closeset Second triangle (index 1)
                Assert.That(intersection.T, Is.EqualTo(5.0).Within(1e-10)); // Distance to z=5
                Assert.That(intersection.Point.Z, Is.EqualTo(5.0).Within(1e-10));
            }
        }

        [Test]
        public void RayMeshIntersects_BoundingBoxCulling()
        {
            // Create a mesh far from the ray
            var vertices = new[]
            {
                new Vector3Double(100, 100, 0),
                new Vector3Double(101, 100, 0),
                new Vector3Double(100, 101, 0)
            };
            var indexes = new[] { 0, 1, 2 };
            var mesh = new Mesh(vertices, indexes);

            // Ray nowhere near the mesh
            var ray = new Ray(new Vector3Double(0, 0, 1), new Vector3Double(0, 0, -1));

            // Should early-out via bounding box test
            bool hit = RayMeshIntersector.Intersects(ray, mesh, out _);

            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayMeshIntersectAll_MultipleHits()
        {
            // Create a mesh with two triangles at different distances
            var vertices = new[]
            {
                // Triangle 1 at z=0
                new Vector3Double(-1, -1, 0),
                new Vector3Double(1, -1, 0),
                new Vector3Double(0, 1, 0),
                // Triangle 2 at z=5
                new Vector3Double(-1, -1, 5),
                new Vector3Double(1, -1, 5),
                new Vector3Double(0, 1, 5)
            };
            var indexes = new[] { 0, 1, 2, 3, 4, 5 };
            var mesh = new Mesh(vertices, indexes);

            // Ray from above, should hit both triangles
            var ray = new Ray(new Vector3Double(0, 0, 10), new Vector3Double(0, 0, -1));

            var intersections = RayMeshIntersector.IntersectAll(ray, mesh);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(intersections, Has.Count.EqualTo(2));
                // Should be sorted by distance
                Assert.That(intersections[0].T, Is.LessThan(intersections[1].T));
                Assert.That(intersections[0].Point.Z, Is.EqualTo(5.0).Within(1e-10)); // Nearer
                Assert.That(intersections[1].Point.Z, Is.EqualTo(0.0).Within(1e-10)); // Farther
            }
        }

        [Test]
        public void RayMeshIntersectAll_NoHits()
        {
            // Create a simple mesh
            var vertices = new[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0)
            };
            var indexes = new[] { 0, 1, 2 };
            var mesh = new Mesh(vertices, indexes);

            // Ray pointing away
            var ray = new Ray(new Vector3Double(0, 0, 1), new Vector3Double(0, 0, 1));

            var intersections = RayMeshIntersector.IntersectAll(ray, mesh);

            Assert.That(intersections, Is.Empty);
        }

        [Test]
        public void RayMeshIntersectsAny_FastExit()
        {
            // Create a mesh with multiple triangles
            var vertices = new[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0),
                new Vector3Double(0, 0, 5),
                new Vector3Double(1, 0, 5),
                new Vector3Double(0, 1, 5)
            };
            var indexes = new[] { 0, 1, 2, 3, 4, 5 };
            var mesh = new Mesh(vertices, indexes);

            // Ray should hit - IntersectsAny should return immediately
            var ray = new Ray(new Vector3Double(0.25, 0.25, 10), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.IntersectsAny(ray, mesh);

            Assert.That(hit, Is.True);
        }

        [Test]
        public void RayMeshIntersectsAny_NoHit()
        {
            // Create a simple mesh
            var vertices = new[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 0, 0),
                new Vector3Double(0, 1, 0)
            };
            var indexes = new[] { 0, 1, 2 };
            var mesh = new Mesh(vertices, indexes);

            // Ray pointing away
            var ray = new Ray(new Vector3Double(0, 0, 1), new Vector3Double(0, 0, 1));

            bool hit = RayMeshIntersector.IntersectsAny(ray, mesh);

            Assert.That(hit, Is.False);
        }

        [Test]
        public void RayMeshIntersects_CubeMesh()
        {
            // Create a simple cube mesh (using 12 triangles for 6 faces)
            var vertices = new[]
            {
                // Bottom face (z=0)
                new Vector3Double(0, 0, 0), new Vector3Double(1, 0, 0),
                new Vector3Double(1, 1, 0), new Vector3Double(0, 1, 0),
                // Top face (z=1)
                new Vector3Double(0, 0, 1), new Vector3Double(1, 0, 1),
                new Vector3Double(1, 1, 1), new Vector3Double(0, 1, 1)
            };
            var indexes = new[]
            {
                // Bottom (2 triangles)
                0, 2, 1,  0, 3, 2,
                // Top (2 triangles)
                4, 5, 6,  4, 6, 7
            };
            var mesh = new Mesh(vertices, indexes);

            // Ray from above, hitting the top face
            var ray = new Ray(new Vector3Double(0.5, 0.5, 2), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.Intersects(ray, mesh, out RayMeshIntersection intersection);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(hit, Is.True);
                Assert.That(intersection.Point.Z, Is.EqualTo(1.0).Within(1e-10));
                Assert.That(intersection.T, Is.EqualTo(1.0).Within(1e-10));
            }
        }

        [Test]
        public void RayMeshIntersects_EmptyMesh()
        {
            // Empty mesh
            var mesh = new Mesh();

            var ray = new Ray(new Vector3Double(0, 0, 1), new Vector3Double(0, 0, -1));

            bool hit = RayMeshIntersector.Intersects(ray, mesh, out _);

            Assert.That(hit, Is.False);
        }
    }
}
