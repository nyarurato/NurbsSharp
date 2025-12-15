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
    public class CurveAnalyzerTest
    {
        [Test]
        public void FindClosestPoint_OnStraightLine_ReturnsPerpendicularPoint()
        {
            // Create a straight line from (0,0,0) to (10,0,0)
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 1.0, 1.0 }, 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point above the line
            var target = new Vector3Double(5, 3, 0);

            // Find closest point using operator
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            // Should find point at (5,0,0)
            Assert.That(point.X, Is.EqualTo(5.0).Within(1e-5), "X coordinate should be 5");
            Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5), "Y coordinate should be 0");
            Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-5), "Z coordinate should be 0");
            Assert.That(distance, Is.EqualTo(3.0).Within(1e-5), "Distance should be 3");
            Assert.That(t, Is.EqualTo(0.5).Within(1e-3), "Parameter should be 0.5");
        }

        [Test]
        public void FindClosestPoint_OnStraightLine_UsingInstanceMethod()
        {
            // Create a straight line from (0,0,0) to (10,0,0)
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 1.0, 1.0 }, 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point above the line
            var target = new Vector3Double(5, 3, 0);

            // Find closest point using instance method
            var (t, point, distance) = curve.FindClosestPoint(target);

            // Should find point at (5,0,0)
            Assert.That(point.X, Is.EqualTo(5.0).Within(1e-5), "X coordinate should be 5");
            Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5), "Y coordinate should be 0");
            Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-5), "Z coordinate should be 0");
            Assert.That(distance, Is.EqualTo(3.0).Within(1e-5), "Distance should be 3");
        }

        [Test]
        public void FindClosestPoint_OnCircularArc_ReturnsRadialPoint()
        {
            // Create a circular arc (90 degree arc in XY plane, radius 5)
            // Using rational quadratic NURBS representation
            double w = Math.Sqrt(2) / 2.0; // weight for 90-degree arc
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(5, 0, 0), 1),
                new ControlPoint(new Vector3Double(5, 5, 0), w),
                new ControlPoint(new Vector3Double(0, 5, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, 2);
            var curve = new NurbsCurve(2, knotVector, controlPoints);

            // Target point at origin (center of circle)
            var target = new Vector3Double(0, 0, 0);

            // Find closest point
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            // Distance should be approximately radius (5.0)
            Assert.That(distance, Is.EqualTo(5.0).Within(1e-3), "Distance should be approximately 5 (radius)");

            // Point should be on the arc
            double radiusCheck = Math.Sqrt(point.X * point.X + point.Y * point.Y);
            Assert.That(radiusCheck, Is.EqualTo(5.0).Within(1e-3), "Point should be on the circle");
        }

        [Test]
        public void FindClosestPoint_WithInitialGuess_ConvergesFaster()
        {
            // Create a straight line from (0,0,0) to (10,0,0)
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 1.0, 1.0 }, 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point
            var target = new Vector3Double(7, 2, 0);

            // Find closest point with good initial guess
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target, initialT: 0.7);

            // Should find point at (7,0,0)
            Assert.That(point.X, Is.EqualTo(7.0).Within(1e-5), "X coordinate should be 7");
            Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5), "Y coordinate should be 0");
            Assert.That(distance, Is.EqualTo(2.0).Within(1e-5), "Distance should be 2");
        }

        [Test]
        public void FindClosestPoint_WithInitialGuess_UsingInstanceMethod()
        {
            // Create a straight line
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 1.0, 1.0 }, 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var target = new Vector3Double(7, 2, 0);

            // Use instance method with initial guess
            var (t, point, distance) = curve.FindClosestPointWithInitialGuess(target, initialT: 0.7);

            Assert.That(point.X, Is.EqualTo(7.0).Within(1e-5));
            Assert.That(distance, Is.EqualTo(2.0).Within(1e-5));
        }

        [Test]
        public void FindClosestPoint_OnQuadraticCurve_FindsCorrectPoint()
        {
            // Create a parabola-like curve
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(5, 10, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, 2);
            var curve = new NurbsCurve(2, knotVector, controlPoints);

            // Target point near the apex
            var target = new Vector3Double(5, 6, 0);

            // Find closest point
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            // Point should be near the apex of the parabola
            Assert.That(point.X, Is.EqualTo(5.0).Within(0.5), "X coordinate should be near 5");
            Assert.That(point.Y, Is.GreaterThan(4.0), "Y coordinate should be significantly positive");
            Assert.That(distance, Is.LessThan(2.0), "Distance should be reasonably small");
        }

        [Test]
        public void FindClosestPoint_OnEndpoints_ReturnsEndpoint()
        {
            // Create a straight line
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 1.0, 1.0 }, 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point beyond the start
            var target = new Vector3Double(-5, 2, 0);

            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            // Should clamp to start point (0,0,0)
            Assert.That(point.X, Is.EqualTo(0.0).Within(1e-5), "Should return start point");
            Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5));
        }

        [Test]
        public void FindClosestPoint_PointOnCurve_ReturnsZeroDistance()
        {
            // Create a straight line
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 1.0, 1.0 }, 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point exactly on the curve
            var target = new Vector3Double(5, 0, 0);

            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            // Distance should be essentially zero
            Assert.That(distance, Is.LessThan(1e-5), "Distance should be near zero");
            Assert.That(point.X, Is.EqualTo(5.0).Within(1e-5));
        }

        [Test]
        public void FindClosestPoint_3DCurve_ReturnsCorrectPoint()
        {
            // Create a 3D curve (helix-like)
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(5, 5, 5), 1),
                new ControlPoint(new Vector3Double(10, 0, 10), 1)
            };
            var knotVector = new KnotVector(new[] { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 }, 2);
            var curve = new NurbsCurve(2, knotVector, controlPoints);

            // Target point in 3D space
            var target = new Vector3Double(5, 2, 5);

            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            // Should find a reasonable closest point
            Assert.That(distance, Is.GreaterThan(0.0), "Distance should be positive");
            Assert.That(distance, Is.LessThan(5.0), "Distance should be reasonable");
            
            // Verify the point is actually on the curve by evaluating at parameter t
            var verification = curve.GetPos(t);
            Assert.That((verification - point).magnitude, Is.LessThan(1e-5), "Returned point should match evaluation at parameter t");
        }
    }
}