using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using NurbsSharp.Generation;
using System;
using System.Linq;

namespace UnitTests.Operation
{
    [TestFixture]
    public class SurfaceOperatorTest
    {
        [Test]
        public void ExtractIsoCurve_PlanarSurface()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var face = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            // Extract U-isocurve at u=0.5 (mid-width, along height)
            // Note: CreateFace creates a surface where:
            // cps[0][0] = p00 (0,0,0)
            // cps[0][1] = p01 (10,0,0) -> V direction is X axis
            // cps[1][0] = p10 (0,10,0) -> U direction is Y axis
            // cps[1][1] = p11 (10,10,0)
            
            // So ExtractIsoCurveU(0.5) fixes U=0.5 (Y=5). Varies V (X).
            // Result is horizontal line at Y=5, from X=0 to X=10.
            var isoU = SurfaceOperator.ExtractIsoCurveU(face, 0.5);
            Assert.That(isoU.Degree, Is.EqualTo(face.DegreeV));
            
            // Check start and end points
            var start = isoU.GetPos(0.0); // v=0 -> X=0
            var end = isoU.GetPos(1.0);   // v=1 -> X=10
            using (Assert.EnterMultipleScope())
            {
                Assert.That(start.X, Is.EqualTo(0.0).Within(1e-6));
                Assert.That(start.Y, Is.EqualTo(5.0).Within(1e-6));
                Assert.That(end.X, Is.EqualTo(10.0).Within(1e-6));
                Assert.That(end.Y, Is.EqualTo(5.0).Within(1e-6));
            }
            // Check sample points along the curve
            var sampleCount = 10;
            for (int i = 0; i <= sampleCount; i++)
            {
                double t = i / (double)sampleCount;
                var ptOnCurve = isoU.GetPos(t);
                var ptOnSurface = face.GetPos(0.5,t); // v=t, u=0.5
                Assert.That(ptOnCurve.DistanceTo(ptOnSurface), Is.LessThan(1e-6));
            }


            // Extract V-isocurve at v=0.25 (quarter-width)
            // Fixes V=0.25 (X=2.5). Varies U (Y).
            // Result is vertical line at X=2.5, from Y=0 to Y=10.
            var isoV = SurfaceOperator.ExtractIsoCurveV(face, 0.25);
            Assert.That(isoV.Degree, Is.EqualTo(face.DegreeU));

            var startV = isoV.GetPos(0.0); // u=0 -> Y=0
            var endV = isoV.GetPos(1.0);   // u=1 -> Y=10

            using (Assert.EnterMultipleScope())
            {
                Assert.That(startV.X, Is.EqualTo(2.5).Within(1e-6));
                Assert.That(startV.Y, Is.EqualTo(0.0).Within(1e-6));
                Assert.That(endV.X, Is.EqualTo(2.5).Within(1e-6));
                Assert.That(endV.Y, Is.EqualTo(10.0).Within(1e-6));
            }
            // Check sample points along the V-isocurve
            for (int i = 0; i <= sampleCount; i++)
            {
                double t = i / (double)sampleCount;
                var ptOnCurve = isoV.GetPos(t);
                var ptOnSurface = face.GetPos(t,0.25); // v=0.25, u=t
                Assert.That(ptOnCurve.DistanceTo(ptOnSurface), Is.LessThan(1e-6));
            }
        }

        [Test]
        public void ExtractIsoCurve_Cylinder()
        {
            double radius = 5.0;
            double height = 10.0;
            var cylinders = PrimitiveFactory.CreateCylinder(radius, height, true);
            var cylinder = cylinders[0]; // First surface is the side

            // Cylinder parameterization in PrimitiveFactory:
            // DegreeU=1 (Linear, along height/axis)
            // DegreeV=2 (Quadratic, circle section)
            // Z range is [-height/2, height/2]
            
            // Extract IsoCurveU at u=0.5 (middle of height). Should be a circle at Z=0.
            // Fix U (height), vary V (angle).
            var circle = SurfaceOperator.ExtractIsoCurveU(cylinder, 0.5);
            Assert.That(circle.Degree, Is.EqualTo(2));
            
            // Check points on the extracted circle
            for(int i=0; i<=10; i++)
            {
                double t = i / 10.0;
                var pt = circle.GetPos(t);
                var surfPt = cylinder.GetPos(0.5, t);
                Assert.That(pt.DistanceTo(surfPt), Is.LessThan(1e-6));
                
                // Radius check (assuming cylinder is along Z axis)
                double r = Math.Sqrt(pt.X*pt.X + pt.Y*pt.Y);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(r, Is.EqualTo(radius).Within(1e-6));
                    Assert.That(pt.Z, Is.EqualTo(0.0).Within(1e-6));
                }
            }
            //check sample points
            var sampleCount = 10;
            for (int i = 0; i <= sampleCount; i++)
            {
                double t = i / (double)sampleCount;
                var ptOnCurve = circle.GetPos(t);
                var ptOnSurface = cylinder.GetPos(0.5, t); // u=0.5, v=t
                Assert.That(ptOnCurve.DistanceTo(ptOnSurface), Is.LessThan(1e-6));
            }

            // Extract IsoCurveV at v=0.0 (seam). Should be a line.
            // Fix V (angle), vary U (height).
            var line = SurfaceOperator.ExtractIsoCurveV(cylinder, 0.0);
            Assert.That(line.Degree, Is.EqualTo(1));
            
            var p1 = line.GetPos(0.0);
            var p2 = line.GetPos(1.0);
            
            // Check if it matches surface points
            var surfP1 = cylinder.GetPos(0.0, 0.0);
            var surfP2 = cylinder.GetPos(1.0, 0.0);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(p1.DistanceTo(surfP1), Is.LessThan(1e-6));
                Assert.That(p2.DistanceTo(surfP2), Is.LessThan(1e-6));

                Assert.That(p1.Z, Is.EqualTo(-height / 2).Within(1e-6));
                Assert.That(p2.Z, Is.EqualTo(height / 2).Within(1e-6));
            }

            // Check sample points along the line
            for (int i = 0; i <= sampleCount; i++)
            {
                double t = i / (double)sampleCount;
                var ptOnCurve = line.GetPos(t);
                var ptOnSurface = cylinder.GetPos(t, 0.0); // u=t, v=0.0
                Assert.That(ptOnCurve.DistanceTo(ptOnSurface), Is.LessThan(1e-6));
            }
        }

        [Test]
        public void ExtractIsoCurve_ConvenienceMethods()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var face = PrimitiveFactory.CreateFace(p00, p01, p10, p11);

            var isoU = face.GetIsoCurveU(0.5);
            var isoV = face.GetIsoCurveV(0.5);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(isoU, Is.Not.Null);
                Assert.That(isoV, Is.Not.Null);
            
                Assert.That(isoU.Degree, Is.EqualTo(face.DegreeV));
                Assert.That(isoV.Degree, Is.EqualTo(face.DegreeU));
            }
        }

        /// <summary>
        /// FindClosestPoint on a flat planar surface returns the projected point.
        /// </summary>
        [Test]
        public void FindClosestPoint_PlanarSurface_ProjectsCorrectly()
        {
            // Flat XY-plane surface from (0,0,0) to (10,10,0)
            var face = PrimitiveFactory.CreateFace(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 0, 0),
                new Vector3Double(0, 10, 0),
                new Vector3Double(10, 10, 0));

            // Target point above the center of the surface
            var target = new Vector3Double(5, 5, 3);
            var result = SurfaceOperator.FindClosestPoint(face, target);

            // The closest point should be the projection onto the plane (z=0)
            using (Assert.EnterMultipleScope())
            {
                Assert.That(result.point.X, Is.EqualTo(5.0).Within(0.01));
                Assert.That(result.point.Y, Is.EqualTo(5.0).Within(0.01));
                Assert.That(result.point.Z, Is.EqualTo(0.0).Within(0.01));
                Assert.That(result.distance, Is.EqualTo(3.0).Within(0.01));
            }
        }

        /// <summary>
        /// FindClosestPoint with initial guess on a planar surface.
        /// </summary>
        [Test]
        public void FindClosestPoint_PlanarSurface_WithInitialGuess()
        {
            var face = PrimitiveFactory.CreateFace(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 0, 0),
                new Vector3Double(0, 10, 0),
                new Vector3Double(10, 10, 0));

            var target = new Vector3Double(2, 8, 1);
            var result = SurfaceOperator.FindClosestPoint(face, target, 0.2, 0.8);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(result.point.X, Is.EqualTo(2.0).Within(0.05));
                Assert.That(result.point.Y, Is.EqualTo(8.0).Within(0.05));
                Assert.That(result.point.Z, Is.EqualTo(0.0).Within(0.05));
                Assert.That(result.distance, Is.EqualTo(1.0).Within(0.05));
            }
        }

        /// <summary>
        /// FindClosestPoint on a sphere surface: the closest point should lie on the sphere.
        /// Also verifies that explicit tolerance/gridDivisions produce the same result as defaults.
        /// </summary>
        [Test]
        public void FindClosestPoint_Sphere_ClosestPointLiesOnSphere()
        {
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);

            // Target point outside the sphere along the X axis
            var target = new Vector3Double(8, 0, 0);

            // With explicit parameters
            var resultExplicit = SurfaceOperator.FindClosestPoint(sphere, target, tolerance: 1e-5, gridDivisions: 6);

            // With default parameters (tolerance=1e-6, gridDivisions=5)
            var resultDefault = SurfaceOperator.FindClosestPoint(sphere, target);

            // Both results should be consistent (both should find a point on the sphere)
            double distExplicit = Math.Sqrt(
                resultExplicit.point.X * resultExplicit.point.X +
                resultExplicit.point.Y * resultExplicit.point.Y +
                resultExplicit.point.Z * resultExplicit.point.Z);
            double distDefault = Math.Sqrt(
                resultDefault.point.X * resultDefault.point.X +
                resultDefault.point.Y * resultDefault.point.Y +
                resultDefault.point.Z * resultDefault.point.Z);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(distExplicit, Is.EqualTo(radius).Within(0.1));
                Assert.That(distDefault,  Is.EqualTo(radius).Within(0.1));
                Assert.That(resultExplicit.distance, Is.EqualTo(3.0).Within(0.2)); // 8 - 5 = 3
                Assert.That(resultDefault.distance,  Is.EqualTo(3.0).Within(0.2));
                // Explicit and default converge to same result within reasonable tolerance
                Assert.That(resultExplicit.point.DistanceTo(resultDefault.point), Is.LessThan(0.5));
            }
        }

        /// <summary>
        /// FindClosestPoint with null surface throws ArgumentNullException.
        /// </summary>
        [Test]
        public void FindClosestPoint_NullSurface_Throws()
        {
            Assert.Throws<ArgumentNullException>(() =>
            {
                SurfaceOperator.FindClosestPoint(null!, new Vector3Double(1, 1, 1));
            });
        }
    }
}
