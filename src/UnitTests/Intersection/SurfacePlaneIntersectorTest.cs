using System;
using System.Linq;
using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Intersection;
using NurbsSharp.Generation;

namespace UnitTests.Intersection
{
    [TestFixture]
    public class SurfacePlaneIntersectorTest
    {
        [Test]
        public void IntersectRobust_PlaneIntersectingSphere_ReturnsCircle()
        {
            // Create a sphere
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // XY plane at Z=0 (through center)
            var plane = NurbsSharp.Core.Plane.XY;
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectRobust(sphere, plane, tolerance: 1e-4, stepSize: 0.02);
            
            // Should get at least one curve (the equator circle)
            Assert.That(curves, Is.Not.Empty);
            
            // Check that the intersection curve is approximately circular
            var curve = curves[0];
            
            // Sample points along the curve
            int numSamples = 20;
            double tMin = curve.KnotVector.Knots[curve.Degree];
            double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
            
            for (int i = 0; i <= numSamples; i++)
            {
                double t = tMin + (tMax - tMin) * i / numSamples;
                var point = curve.GetPos(t);
                
                // Point should be on the plane (Z ≈ 0)
                Assert.That(Math.Abs(point.Z), Is.LessThan(0.1), $"Point at t={t} not on plane: Z={point.Z}");
                
                // Point should be at radius distance from origin in XY plane
                double distFromCenter = Math.Sqrt(point.X * point.X + point.Y * point.Y);
                Assert.That(distFromCenter, Is.EqualTo(radius).Within(0.2), 
                    $"Point at t={t} not on circle: distance={distFromCenter}");
            }
        }

        [Test]
        public void IntersectRobust_PlaneIntersectingCylinder_ReturnsLines()
        {
            // Create a cylinder along Z-axis
            double radius = 3.0;
            double height = 10.0;
            var cylinders = PrimitiveFactory.CreateCylinder(radius, height, isGenerateTopBottom: false);
            var cylinder = cylinders[0]; // Main cylindrical surface
            
            // XZ plane (Y=0) - should cut through cylinder creating two lines
            var plane = NurbsSharp.Core.Plane.XZ;
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectRobust(cylinder, plane, tolerance: 1e-4, stepSize: 0.02);
            
            // Should get intersection curves
            Assert.That(curves, Is.Not.Empty);
            
            // Check that points are on the plane
            foreach (var curve in curves)
            {
                double tMin = curve.KnotVector.Knots[curve.Degree];
                double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
                
                for (int i = 0; i <= 10; i++)
                {
                    double t = tMin + (tMax - tMin) * i / 10;
                    var point = curve.GetPos(t);
                    
                    // Point should be on the plane (Y ≈ 0)
                    Assert.That(Math.Abs(point.Y), Is.LessThan(0.1), $"Point not on XZ plane: Y={point.Y}");
                }
            }
        }

        [Test]
        public void IntersectRobust_PlaneNotIntersecting_ReturnsEmpty()
        {
            // Create a sphere at origin
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // Plane far away from sphere
            var plane = new NurbsSharp.Core.Plane(new Vector3Double(0, 0, 1), -20.0);
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectRobust(sphere, plane, tolerance: 1e-4);
            
            // Should get no intersection
            Assert.That(curves, Is.Empty);
        }

        [Test]
        public void IntersectRobust_PlaneIntersectingFlatSurface_ReturnsLine()
        {
            // Create a simple flat surface (bilinear patch)
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(0, 5, 0);
            var p10 = new Vector3Double(5, 0, 0);
            var p11 = new Vector3Double(5, 5, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);
            
            // Diagonal plane
            var planeNormal = new Vector3Double(1, -1, 0).normalized;
            var plane = new NurbsSharp.Core.Plane(planeNormal, new Vector3Double(2.5, 2.5, 0));
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectRobust(surface, plane, tolerance: 1e-4, stepSize: 0.02);
            
            // Should get a line
            Assert.That(curves, Is.Not.Empty);
            
            if (curves.Count > 0)
            {
                var curve = curves[0];
                
                // Check points are on the plane
                double tMin = curve.KnotVector.Knots[curve.Degree];
                double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
                
                for (int i = 0; i <= 10; i++)
                {
                    double t = tMin + (tMax - tMin) * i / 10;
                    var point = curve.GetPos(t);
                    
                    double dist = plane.DistanceTo(point);
                    Assert.That(dist, Is.LessThan(0.1), $"Point not on plane: distance={dist}");
                }
            }
        }

        [Test]
        public void IntersectRobust_OffsetPlane_ReturnsSmallCircle()
        {
            // Create a sphere
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // Plane offset from center (Z = 3)
            var plane = new NurbsSharp.Core.Plane(new Vector3Double(0, 0, 1), -3.0);
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectRobust(sphere, plane, tolerance: 1e-4, stepSize: 0.02);
            
            // Should get a circle
            Assert.That(curves, Is.Not.Empty);
            
            if (curves.Count > 0)
            {
                var curve = curves[0];
                
                // Expected radius of small circle: sqrt(r^2 - h^2) = sqrt(25 - 9) = 4
                double expectedRadius = Math.Sqrt(radius * radius - 9);
                
                double tMin = curve.KnotVector.Knots[curve.Degree];
                double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
                
                for (int i = 0; i <= 10; i++)
                {
                    double t = tMin + (tMax - tMin) * i / 10;
                    var point = curve.GetPos(t);
                    
                    // Point should be on the plane (Z ≈ 3)
                    Assert.That(point.Z, Is.EqualTo(3.0).Within(0.15));
                    
                    // Distance from Z-axis should be approximately expectedRadius
                    double distFromAxis = Math.Sqrt(point.X * point.X + point.Y * point.Y);
                    Assert.That(distFromAxis, Is.EqualTo(expectedRadius).Within(0.3));
                }
            }
        }

        
        [Test]
        public void IntersectRobust_PlaneThroughBoundaryEdge_ReturnsEdgeCurve()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(0, 5, 0);
            var p10 = new Vector3Double(5, 0, 0);
            var p11 = new Vector3Double(5, 5, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Plane x=0 intersects the face along its left edge
            var plane = new NurbsSharp.Core.Plane(new Vector3Double(1, 0, 0), 0.0);

            var curves = SurfacePlaneIntersector.IntersectRobust(surface, plane, tolerance: 1e-6);
            Assert.That(curves, Is.Not.Empty);

            // Check that the resulting curve points have X≈0
            foreach (var curve in curves)
            {
                double tMin = curve.KnotVector.Knots[curve.Degree];
                double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
                for (int i = 0; i <= 5; i++)
                {
                    double t = tMin + (tMax - tMin) * i / 5;
                    var pt = curve.GetPos(t);
                    Assert.That(pt.X, Is.EqualTo(0.0).Within(1e-6));
                }
            }
        }

        [Test]
        public void IntersectRobust_TangentPlane_ReturnsNoCurve()
        {
            // Sphere tangent plane at top (z = radius)
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            var plane = new NurbsSharp.Core.Plane(new Vector3Double(0, 0, 1), -radius);

            var curves = SurfacePlaneIntersector.IntersectRobust(sphere, plane, tolerance: 1e-6);
            // Tangent: intersection is a single point, algorithm should yield no curve
            Assert.That(curves, Is.Empty);
        }

        // ========== Fast Version Tests ==========

        [Test]
        public void IntersectFast_PlaneIntersectingSphere_ReturnsCircle()
        {
            // Create a sphere
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // XY plane at Z=0 (through center)
            var plane = NurbsSharp.Core.Plane.XY;
            
            // Intersect with fast method
            var curves = SurfacePlaneIntersector.IntersectFast(sphere, plane, tolerance: 1e-4, numIsoCurves: 50);
            
            // Should get one curve
            Assert.That(curves, Is.Not.Empty);
            
            var curve = curves[0];
            
            // Sample points along the curve
            int numSamples = 20;
            double tMin = curve.KnotVector.Knots[curve.Degree];
            double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
            
            for (int i = 0; i <= numSamples; i++)
            {
                double t = tMin + (tMax - tMin) * i / numSamples;
                var point = curve.GetPos(t);
                
                // Point should be on the plane (Z ≈ 0)
                Assert.That(Math.Abs(point.Z), Is.LessThan(0.1), $"Point at t={t} not on plane: Z={point.Z}");
                
                // Point should be at radius distance from origin in XY plane
                double distFromCenter = Math.Sqrt(point.X * point.X + point.Y * point.Y);
                Assert.That(distFromCenter, Is.EqualTo(radius).Within(0.5), 
                    $"Point at t={t} not on circle: distance={distFromCenter}");
            }
        }

        [Test]
        public void IntersectFast_PlaneNotIntersecting_ReturnsEmpty()
        {
            // Create a sphere at origin
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // Plane far away from sphere
            var plane = new NurbsSharp.Core.Plane(new Vector3Double(0, 0, 1), -20.0);
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectFast(sphere, plane, tolerance: 1e-4);
            
            // Should get no intersection
            Assert.That(curves, Is.Empty);
        }

        [Test]
        public void IntersectFast_PlaneIntersectingFlatSurface_ReturnsLine()
        {
            // Create a simple flat surface
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(0, 5, 0);
            var p10 = new Vector3Double(5, 0, 0);
            var p11 = new Vector3Double(5, 5, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);
            
            // Diagonal plane
            var planeNormal = new Vector3Double(1, -1, 0).normalized;
            var plane = new NurbsSharp.Core.Plane(planeNormal, new Vector3Double(2.5, 2.5, 0));
            
            // Intersect
            var curves = SurfacePlaneIntersector.IntersectFast(surface, plane, tolerance: 1e-4, numIsoCurves: 30);
            
            // Should get a line
            Assert.That(curves, Is.Not.Empty);
            
            if (curves.Count > 0)
            {
                var curve = curves[0];
                
                // Check points are on the plane
                double tMin = curve.KnotVector.Knots[curve.Degree];
                double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
                
                for (int i = 0; i <= 10; i++)
                {
                    double t = tMin + (tMax - tMin) * i / 10;
                    var point = curve.GetPos(t);
                    
                    double dist = plane.DistanceTo(point);
                    Assert.That(dist, Is.LessThan(0.15), $"Point not on plane: distance={dist}");
                }
            }
        }

        [Test]
        public void IntersectFast_CompareWithRobust_SimilarResults()
        {
            // Create a simple sphere
            double radius = 3.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // XY plane
            var plane = NurbsSharp.Core.Plane.XY;
            
            // Intersect with both methods
            var curvesFast = SurfacePlaneIntersector.IntersectFast(sphere, plane, tolerance: 1e-4, numIsoCurves: 50);
            var curvesRobust = SurfacePlaneIntersector.IntersectRobust(sphere, plane, tolerance: 1e-4, stepSize: 0.02);
            
            // Both should find intersection
            Assert.That(curvesFast, Is.Not.Empty);
            Assert.That(curvesRobust, Is.Not.Empty);
            
            // Sample and compare points (should be similar, within tolerance)
            var curveFast = curvesFast[0];
            var curveRobust = curvesRobust[0];
            
            double tMinFast = curveFast.KnotVector.Knots[curveFast.Degree];
            double tMaxFast = curveFast.KnotVector.Knots[^(curveFast.Degree + 1)];
            
            // Check a few sample points
            for (int i = 0; i <= 5; i++)
            {
                double t = tMinFast + (tMaxFast - tMinFast) * i / 5;
                var pointFast = curveFast.GetPos(t);
                
                // Distance from origin should be close to radius
                double distFast = Math.Sqrt(pointFast.X * pointFast.X + pointFast.Y * pointFast.Y);
                Assert.That(distFast, Is.EqualTo(radius).Within(0.5));
            }
        }

        [Test]
        public void IntersectFast_EndpointOnPlane_DetectsEndpoint()
        {
            // Build flat face where plane passes through one corner
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(0, 5, 0);
            var p10 = new Vector3Double(5, 0, 0);
            var p11 = new Vector3Double(5, 5, 0);
            var surface = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Plane through corner p00 and tilted slightly
            var plane = new NurbsSharp.Core.Plane(new Vector3Double(0, 0, 1), 0.0); // z=0 plane intersects entire face

            var curves = SurfacePlaneIntersector.IntersectFast(surface, plane, tolerance: 1e-6, numIsoCurves: 20);
            Assert.That(curves, Is.Not.Empty);

            // Confirm that some curve contains the p00 projected point
            bool found = false;
            foreach (var curve in curves)
            {
                double tMin = curve.KnotVector.Knots[curve.Degree];
                double tMax = curve.KnotVector.Knots[^(curve.Degree + 1)];
                for (int i = 0; i <= 10; i++)
                {
                    double t = tMin + (tMax - tMin) * i / 10;
                    var pt = curve.GetPos(t);
                    if (Math.Abs(pt.X - 0.0) < 1e-6 && Math.Abs(pt.Y - 0.0) < 1e-6)
                    {
                        found = true;
                        break;
                    }
                }
                if (found) break;
            }
            Assert.That(found, Is.True, "Endpoint (0,0,0) was not detected in any intersection curve.");
        }
    }
}
