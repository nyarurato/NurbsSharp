using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using NurbsSharp.Generation;
using System;
using System.Linq;
using NurbsSharp.Analysis;

namespace UnitTests.Analysis
{
    [TestFixture]
    public class SurfaceAnalyzerTest
    {
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

            // Should find point at (5,5,0) - directly below target
            Assert.That(point.X, Is.EqualTo(5.0).Within(1e-3), "X coordinate should be 5");
            Assert.That(point.Y, Is.EqualTo(5.0).Within(1e-3), "Y coordinate should be 5");
            Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-3), "Z coordinate should be 0");
            Assert.That(distance, Is.EqualTo(3.0).Within(1e-3), "Distance should be 3");
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

            // Should find point at (7,3,0)
            Assert.That(point.X, Is.EqualTo(7.0).Within(1e-3), "X coordinate should be 7");
            Assert.That(point.Y, Is.EqualTo(3.0).Within(1e-3), "Y coordinate should be 3");
            Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-3), "Z coordinate should be 0");
            Assert.That(distance, Is.EqualTo(2.0).Within(1e-3), "Distance should be 2");
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
            Assert.That(radiusCheck, Is.EqualTo(radius).Within(1e-2), "Point should be on the cylindrical surface");
            Assert.That(Math.Abs(point.Z), Is.LessThan(height / 2 + 0.1), "Z should be within cylinder height");
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

            // Should find point at (8,6,0)
            Assert.That(point.X, Is.EqualTo(8.0).Within(1e-3), "X coordinate should be 8");
            Assert.That(point.Y, Is.EqualTo(6.0).Within(1e-3), "Y coordinate should be 6");
            Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-3), "Z coordinate should be 0");
            Assert.That(distance, Is.EqualTo(4.0).Within(1e-3), "Distance should be 4");
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

            Assert.That(point.X, Is.EqualTo(8.0).Within(1e-3));
            Assert.That(point.Y, Is.EqualTo(6.0).Within(1e-3));
            Assert.That(distance, Is.EqualTo(4.0).Within(1e-3));
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

            // Distance should be essentially zero
            Assert.That(distance, Is.LessThan(1e-4), "Distance should be near zero");
            Assert.That(point.X, Is.EqualTo(5.0).Within(1e-3));
            Assert.That(point.Y, Is.EqualTo(5.0).Within(1e-3));
        }
    }
}