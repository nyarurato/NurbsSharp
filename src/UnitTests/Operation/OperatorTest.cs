using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.IO.IGES;
using NurbsSharp.Generation;

namespace UnitTests.Operation
{
    internal class OperatorTest
    {
        [Test]
        public void KnotOperator_Insert_TestA()
        {
            // Circle
            int R = 2;
            int degree = 2;
            double[] knots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(R ,  0, 0, 1),
                new ControlPoint(R ,  R, 0, 0.70710678),
                new ControlPoint(0 ,  R, 0, 1),
                new ControlPoint(-R,  R, 0, 0.70710678),
                new ControlPoint(-R,  0, 0, 1),
                new ControlPoint(-R, -R, 0, 0.70710678),
                new ControlPoint(0 , -R, 0, 1),
                new ControlPoint(R , -R, 0, 0.70710678),
                new ControlPoint(R ,  0, 0, 1),

            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            double[] samplePoints = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];
            double[] insert_u = [0.5, 0.3, 0.8, 0.111, 0.2, 0.9999];

            double[] new_knots = knots;
            ControlPoint[] new_cps = controlPoints;
            int insert_count = 0;
            foreach (var u in insert_u)
            {
                (new_knots, new_cps) = KnotOperator.InsertKnot(degree, new_knots, new_cps, u, 1);

                var new_curve = new NurbsCurve(degree, new KnotVector(new_knots, degree), new_cps);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(new_knots, Has.Length.EqualTo(knots.Length + insert_count + 1));
                    Assert.That(new_cps, Has.Length.EqualTo(controlPoints.Length + insert_count + 1));
                }
                foreach (var s in samplePoints)
                {
                    var pos_original = curve.GetPos(s);
                    var pos_new = new_curve.GetPos(s);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                        Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                        Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                    }
                }
                insert_count++;
            }
        }

        [Test]
        public void KnotOperator_Insert_TestB()
        {
            // Line
            int degree = 1;
            double[] knots = [0, 0, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0 ,  0, 0, 1),
                new ControlPoint(10 , 0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];
            double[] insert_u = [0.5, 0.3, 0.8, 0.111, 0.2, 0.9999];
            double[] new_knots = knots;
            ControlPoint[] new_cps = controlPoints;
            int insert_count = 2;
            foreach (var u in insert_u)
            {
                var new_curve = KnotOperator.InsertKnot(curve,u,insert_count);
                new_knots = new_curve.KnotVector.Knots;
                new_cps = new_curve.ControlPoints;
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(new_knots, Has.Length.EqualTo(knots.Length + insert_count));
                    Assert.That(new_cps, Has.Length.EqualTo(controlPoints.Length + insert_count));
                }
                foreach (var s in samplePoints)
                {
                    var pos_original = curve.GetPos(s);
                    var pos_new = new_curve.GetPos(s);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                        Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                        Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                    }
                }
                insert_count++;
            }
        }

        [Test]
        public void KnotOperator_Insert_TestC()
        {
            // Line
            int degree = 1;
            double[] knots = [0, 0, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0 ,  0, 0, 1),
                new ControlPoint(10 , 0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];
            double[] insert_u = [0.5, 0.3, 0.8, 0.111, 0.2, 0.9999];
            double[] new_knots = knots;
            ControlPoint[] new_cps = controlPoints;
            int insert_count = 0;
            foreach (var u in insert_u)
            {
                (new_knots, new_cps) = KnotOperator.InsertKnot(degree, new_knots, new_cps, u, 1);
                var new_curve = new NurbsCurve(degree, new KnotVector(new_knots, degree), new_cps);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(new_knots, Has.Length.EqualTo(knots.Length + insert_count + 1));
                    Assert.That(new_cps, Has.Length.EqualTo(controlPoints.Length + insert_count + 1));
                }
                foreach (var s in samplePoints)
                {
                    var pos_original = curve.GetPos(s);
                    var pos_new = new_curve.GetPos(s);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                        Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                        Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                    }
                }
                insert_count++;
            }
        }

        [Test]
        public void RefineKnot_Line_DuplicatesAndEndpointsIgnored()
        {
            int degree = 1;
            double[] knots = [0, 0, 1, 1];
            ControlPoint[] cps = [
                new ControlPoint(0,0,0,1),
                new ControlPoint(10,0,0,1)
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), cps);

            // Attempt to refine with duplicates and endpoints
            double[] refine = [0.5, 0.5, 0.5, 0.0, 1.0, 0.25];// 0.5:3=>1 ,0.0:ignored, 1.0:ignored, 0.25:1
            var refined = KnotOperator.RefineKnot(curve, refine);

            using (Assert.EnterMultipleScope())
            {
                // Distinct internal knots added should be 0.25 and 0.5 (each once), ignore over degree multiplicities and endpoints
                Assert.That(refined.KnotVector.Knots, Has.Length.EqualTo(knots.Length + 2));
                Assert.That(refined.ControlPoints, Has.Length.EqualTo(cps.Length + 2));
            }

            // Shape preservation: sample points
            double[] samples = [0.0, 0.1, 0.25, 0.4, 0.5, 0.6, 0.75, 0.9, 1.0];
            foreach (var u in samples)
            {
                var p0 = curve.GetPos(u);
                var p1 = refined.GetPos(u);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(p0.X, Is.EqualTo(p1.X).Within(1e-8));
                    Assert.That(p0.Y, Is.EqualTo(p1.Y).Within(1e-8));
                    Assert.That(p0.Z, Is.EqualTo(p1.Z).Within(1e-8));
                }
            }

            // Multiplicity checks
            int multStart = refined.KnotVector.Knots.Count(k => LinAlg.ApproxEqual(0.0, k));
            int multEnd = refined.KnotVector.Knots.Count(k => LinAlg.ApproxEqual(1, k));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(multStart, Is.EqualTo(degree + 1));
                Assert.That(multEnd, Is.EqualTo(degree + 1));
            }
        }

        [Test]
        public void RefineKnot_Circle_AddInteriorDistinctOnly()
        {
            int R = 2;
            int degree = 2;
            double[] knots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[] cps = [
                new ControlPoint(R ,  0, 0, 1),
                new ControlPoint(R ,  R, 0, 0.70710678),
                new ControlPoint(0 ,  R, 0, 1),
                new ControlPoint(-R,  R, 0, 0.70710678),
                new ControlPoint(-R,  0, 0, 1),
                new ControlPoint(-R, -R, 0, 0.70710678),
                new ControlPoint(0 , -R, 0, 1),
                new ControlPoint(R , -R, 0, 0.70710678),
                new ControlPoint(R ,  0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), cps);
            double[] refine = [0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 0.125, 0.375, 0.625, 0.875, 0.0, 1.0];
            var refined = KnotOperator.RefineKnot(curve, refine);

            using (Assert.EnterMultipleScope())
            {
                // New distinct interior values inserted: 0.125,0.375,0.625,0.875 => 4 new knots
                Assert.That(refined.KnotVector.Knots, Has.Length.EqualTo(knots.Length + 4));
                Assert.That(refined.ControlPoints, Has.Length.EqualTo(cps.Length + 4));
            }

            double[] samples = [0, 0.05, 0.125, 0.2, 0.3, 0.375, 0.45, 0.5, 0.6, 0.625, 0.7, 0.75, 0.8, 0.875, 0.95, 1.0];
            foreach (var u in samples)
            {
                var p0 = curve.GetPos(u);
                var p1 = refined.GetPos(u);
                Assert.That(p0.DistanceTo(p1), Is.LessThanOrEqualTo(5e-8));
            }

            // Check interior multiplicities do not exceed degree
            foreach (double val in new double[] { 0.125, 0.375, 0.625, 0.875, 0.25, 0.5, 0.75 })
            {
                int mult = refined.KnotVector.Knots.Count(k => Math.Abs(k - val) < 1e-12);
                Assert.That(mult, Is.LessThanOrEqualTo(degree));
            }
            int multStart = refined.KnotVector.Knots.Count(k => Math.Abs(k - 0.0) < 1e-12);
            int multEnd = refined.KnotVector.Knots.Count(k => Math.Abs(k - 1.0) < 1e-12);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(multStart, Is.EqualTo(degree + 1));
                Assert.That(multEnd, Is.EqualTo(degree + 1));
            }
        }

        [Test]
        public void KnotOperator_InsertKnot_Surface_UDirection()
        {
            // Create a simple planar surface
            int degreeU = 2;
            int degreeV = 1;
            double[] knotsU = [0, 0, 0, 1, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(0, 1, 0, 1)
            ];
            controlPoints[1] = [
                new ControlPoint(1, 0, 0, 1),
                new ControlPoint(1, 1, 0, 1)
            ];
            controlPoints[2] = [
                new ControlPoint(2, 0, 0, 1),
                new ControlPoint(2, 1, 0, 1)
            ];

            var surface = new NurbsSurface(degreeU, degreeV, 
                new KnotVector(knotsU, degreeU), 
                new KnotVector(knotsV, degreeV), 
                controlPoints);

            // Insert knot in U direction
            double u = 0.5;
            int times = 1;
            var newSurface = KnotOperator.InsertKnot(surface, u, times, SurfaceDirection.U);

            // Verify knot vector and control points size
            using (Assert.EnterMultipleScope())
            {
                Assert.That(newSurface.KnotVectorU.Knots, Has.Length.EqualTo(knotsU.Length + times));
                Assert.That(newSurface.ControlPoints, Has.Length.EqualTo(controlPoints.Length + times));
                Assert.That(newSurface.ControlPoints[0], Has.Length.EqualTo(controlPoints[0].Length));
            }

            // Verify shape preservation by sampling
            double[] samplesU = [0.0, 0.25, 0.5, 0.75, 1.0];
            double[] samplesV = [0.0, 0.5, 1.0];
            
            foreach (var su in samplesU)
            {
                foreach (var sv in samplesV)
                {
                    var pos_original = surface.GetPos(su, sv);
                    var pos_new = newSurface.GetPos(su, sv);
                    
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(1e-8));
                        Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(1e-8));
                        Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void KnotOperator_InsertKnot_Surface_VDirection()
        {
            // Create a simple planar surface
            int degreeU = 1;
            int degreeV = 2;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 0, 1, 1, 1];
            
            ControlPoint[][] controlPoints = new ControlPoint[2][];
            controlPoints[0] = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(0, 1, 0, 1),
                new ControlPoint(0, 2, 0, 1)
            ];
            controlPoints[1] = [
                new ControlPoint(1, 0, 0, 1),
                new ControlPoint(1, 1, 0, 1),
                new ControlPoint(1, 2, 0, 1)
            ];

            var surface = new NurbsSurface(degreeU, degreeV, 
                new KnotVector(knotsU, degreeU), 
                new KnotVector(knotsV, degreeV), 
                controlPoints);

            // Insert knot in V direction
            double v = 0.5;
            int times = 1;
            var newSurface = KnotOperator.InsertKnot(surface, v, times, SurfaceDirection.V);

            // Verify knot vector and control points size
            using (Assert.EnterMultipleScope())
            {
                Assert.That(newSurface.KnotVectorV.Knots, Has.Length.EqualTo(knotsV.Length + times));
                Assert.That(newSurface.ControlPoints, Has.Length.EqualTo(controlPoints.Length));
                Assert.That(newSurface.ControlPoints[0], Has.Length.EqualTo(controlPoints[0].Length + times));
            }

            // Verify shape preservation by sampling
            double[] samplesU = [0.0, 0.5, 1.0];
            double[] samplesV = [0.0, 0.25, 0.5, 0.75, 1.0];
            
            foreach (var su in samplesU)
            {
                foreach (var sv in samplesV)
                {
                    var pos_original = surface.GetPos(su, sv);
                    var pos_new = newSurface.GetPos(su, sv);
                    
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(1e-8));
                        Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(1e-8));
                        Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void KnotOperator_InsertKnot_Surface_CurvedSurface()
        {
            // Create a curved surface (cylindrical)
            int degreeU = 2;
            int degreeV = 1;
            double[] knotsU = [0, 0, 0, 1, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            
            double R = 2.0;
            double w = 1.0 / Math.Sqrt(2.0);
            
            ControlPoint[][] controlPoints = new ControlPoint[3][];
            controlPoints[0] = [
                new ControlPoint(R, 0, 0, 1),
                new ControlPoint(R, 0, 1, 1)
            ];
            controlPoints[1] = [
                new ControlPoint(R, R, 0, w),
                new ControlPoint(R, R, 1, w)
            ];
            controlPoints[2] = [
                new ControlPoint(0, R, 0, 1),
                new ControlPoint(0, R, 1, 1)
            ];

            var surface = new NurbsSurface(degreeU, degreeV, 
                new KnotVector(knotsU, degreeU), 
                new KnotVector(knotsV, degreeV), 
                controlPoints);

            // Insert knots in both directions
            double u = 0.5;
            var surfaceU = KnotOperator.InsertKnot(surface, u, 2, SurfaceDirection.U);
            
            double v = 0.5;
            var surfaceUV = KnotOperator.InsertKnot(surfaceU, v, 1, SurfaceDirection.V);

            // Verify knot vector sizes
            using (Assert.EnterMultipleScope())
            {
                Assert.That(surfaceU.KnotVectorU.Knots, Has.Length.EqualTo(knotsU.Length + 2));
                Assert.That(surfaceUV.KnotVectorU.Knots, Has.Length.EqualTo(knotsU.Length + 2));
                Assert.That(surfaceUV.KnotVectorV.Knots, Has.Length.EqualTo(knotsV.Length + 1));
            }

            // Verify shape preservation
            double[] samplesU = [0.0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0];
            double[] samplesV = [0.0, 0.3, 0.5, 0.7, 1.0];
            
            foreach (var su in samplesU)
            {
                foreach (var sv in samplesV)
                {
                    var pos_original = surface.GetPos(su, sv);
                    var pos_afterU = surfaceU.GetPos(su, sv);
                    var pos_afterUV = surfaceUV.GetPos(su, sv);
                    
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_afterU.X, Is.EqualTo(pos_original.X).Within(1e-8));
                        Assert.That(pos_afterU.Y, Is.EqualTo(pos_original.Y).Within(1e-8));
                        Assert.That(pos_afterU.Z, Is.EqualTo(pos_original.Z).Within(1e-8));
                        
                        Assert.That(pos_afterUV.X, Is.EqualTo(pos_original.X).Within(1e-8));
                        Assert.That(pos_afterUV.Y, Is.EqualTo(pos_original.Y).Within(1e-8));
                        Assert.That(pos_afterUV.Z, Is.EqualTo(pos_original.Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void KnotOperator_RemoveKnot_SimpleLine()
        {
            // Line with inserted knot
            int degree = 1;
            double[] knots = [0, 0, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(10, 0, 0, 1)
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            
            // Insert a knot first
            curve = KnotOperator.InsertKnot(curve, 0.5, 1);// add knot at 0.5
            int originalCPCount = curve.ControlPoints.Length;
            
            // Then remove it
            var (success, newCurve) = KnotOperator.RemoveKnot(curve, 0.5);
            
            using (Assert.EnterMultipleScope())
            {
                Assert.That(success, Is.True);
                Assert.That(newCurve, Is.Not.Null);
                Assert.That(newCurve!.ControlPoints, Has.Length.EqualTo(originalCPCount - 1));
                Assert.That(newCurve.KnotVector.Knots, Has.Length.EqualTo(curve.KnotVector.Knots.Length - 1));
            }
            
            // Verify shape preservation
            double[] samples = [0.0, 0.25, 0.5, 0.75, 1.0];
            foreach (var u in samples)
            {
                var pos_original = curve.GetPos(u);
                var pos_removed = newCurve!.GetPos(u);
                
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pos_removed.X, Is.EqualTo(pos_original.X).Within(1e-6));
                    Assert.That(pos_removed.Y, Is.EqualTo(pos_original.Y).Within(1e-6));
                    Assert.That(pos_removed.Z, Is.EqualTo(pos_original.Z).Within(1e-6));
                }
            }
        }

        [Test]
        public void KnotOperator_RemoveKnot_EndpointFails()
        {
            int degree = 2;
            double[] knots = [0, 0, 0, 0.5, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 0),
                new ControlPoint(2, 1, 0),
                new ControlPoint(3, 0, 0)
            ];
            
            // Try to remove endpoint knot (should fail)
            var (success, _, _) = KnotOperator.RemoveKnot(degree, knots, controlPoints, 0.0);
            
            Assert.That(success, Is.False);
        }

        [Test]
        public void KnotOperator_RemoveKnot_NonexistentKnotFails()
        {
            int degree = 2;
            double[] knots = [0, 0, 0, 0.5, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 0),
                new ControlPoint(2, 1, 0),
                new ControlPoint(3, 0, 0)
            ];
            
            var (success, _, _) = KnotOperator.RemoveKnot(degree, knots, controlPoints, 0.75);  // Doesn't exist
            
            Assert.That(success, Is.False);
        }

        [Test]
        public void KnotOperator_RemoveKnot_Degree3Curve()
        {
            int degree = 3;
            double[] knots = [0, 0, 0, 0, 0.5, 1, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 2, 0),
                new ControlPoint(2, 2, 0),
                new ControlPoint(3, 0, 0),
                new ControlPoint(4, -1, 0)
            ];
            // Insert knot first
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            var insert_curve = KnotOperator.InsertKnot(curve, 0.75, 2);// insert knot at 0.75 twice
            insert_curve = KnotOperator.InsertKnot(insert_curve, 0.1, 2);// insert knot at 0.25 twice
            // knot length: +4, cp length: +4

            // Now remove it
            var (success, newCurve) = KnotOperator.RemoveKnot(insert_curve, 0.75);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(success, Is.True);
                Assert.That(newCurve, Is.Not.Null);
                Assert.That(newCurve!.ControlPoints, Has.Length.EqualTo(insert_curve.ControlPoints.Length - 1));
                Assert.That(newCurve.KnotVector.Knots, Has.Length.EqualTo(insert_curve.KnotVector.Knots.Length - 1));
            }

            // Verify shape preservation
            double[] samples = [0.0, 0.25, 0.5, 0.75, 1.0];
            foreach (var u in samples)
            {
                var pos_original = curve.GetPos(u);
                var pos_removed = newCurve!.GetPos(u);
                
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pos_removed.X, Is.EqualTo(pos_original.X).Within(1e-6));
                    Assert.That(pos_removed.Y, Is.EqualTo(pos_original.Y).Within(1e-6));
                    Assert.That(pos_removed.Z, Is.EqualTo(pos_original.Z).Within(1e-6));
                }
            }

        }

        [Test]
        public void KnotOperator_RemoveKnot_Circle()
        {
            double R = 2.0;
            var curve = PrimitiveFactory.CreateCircle(R);

            // Insert knots first
            var refinedCurve = KnotOperator.RefineKnot(curve, [0.125, 0.375, 0.625, 0.875]);

            //remove knots
            double[] knotsToRemove = [0.125, 0.375, 0.625, 0.875];
            double[] samplePoints = [0, 0.05, 0.125, 0.2, 0.3, 0.375, 0.45, 0.5, 0.6, 0.625, 0.7, 0.75, 0.8, 0.875, 0.95, 1.0];

            foreach (var u in knotsToRemove)
            {
                var (success, newCurve) = KnotOperator.RemoveKnot(refinedCurve, u, 1E-06);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(success, Is.True);
                    Assert.That(newCurve, Is.Not.Null);

                }

                foreach(var s in samplePoints)
                {
                    var pos_original = curve.GetPos(s);
                    var pos_new = newCurve!.GetPos(s);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(1e-6));
                        Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(1e-6));
                        Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(1e-6));
                    }
                }

                refinedCurve = newCurve!;
            }

            var remove_curve = KnotOperator.RemoveKnot(refinedCurve, 0.5, 1E-06);
            Assert.That(remove_curve.success, Is.True);

            foreach (var s in samplePoints)
            {
                var pos_original = curve.GetPos(s);
                var pos_new = remove_curve.curve!.GetPos(s);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(0.2));//looser tolerance after lost original shape
                    Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(0.2));
                    Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(0.2));
                }
            }
        }

        [Test]
        public void KnotOperator_RemoveKnot_PlaneSurf(){
            Vector3Double p0 = new Vector3Double(0,0,0);
            Vector3Double p1 = new Vector3Double(10,0,0);
            Vector3Double p2 = new Vector3Double(0,10,0);
            Vector3Double p3 = new Vector3Double(10,10,0);

            var surf = PrimitiveFactory.CreateFace(p0, p1, p2, p3);
            //insert knots first
            var refinedSurf = KnotOperator.RefineKnot(surf, [0.25,0.5,0.75], SurfaceDirection.U);

            //remove knots
            double[] knotsToRemove = [0.25,0.5,0.75];
            double[] uSamples = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
            double[] vSamples = [0.0, 0.3, 0.5, 0.7, 1.0];
            foreach(var u in knotsToRemove){
                var (success, newSurf) = KnotOperator.RemoveKnot(refinedSurf, u, SurfaceDirection.U, 1E-06);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(success, Is.True);
                    Assert.That(newSurf, Is.Not.Null);
                }
                foreach (var su in uSamples)
                {
                    foreach (var sv in vSamples)
                    {
                        var pos_original = surf.GetPos(su, sv);
                        var pos_new = newSurf!.GetPos(su, sv);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(1e-6));
                            Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(1e-6));
                            Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(1e-6));
                        }
                    }
                }
                refinedSurf = newSurf!;
            }
        }

        [Test]
        public void KnotOperator_RemoveKnot_Sphere()
        {
            var surf = PrimitiveFactory.CreateSphere(10.0);

            //remove knots
            double[] knotsToRemove = [ 0.5 ];
            double[] uSamples = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
            double[] vSamples = [0.0, 0.3, 0.5, 0.7, 1.0];

            foreach (var u in knotsToRemove)
            {
                var (success, newSurf) = KnotOperator.RemoveKnot(surf, u, SurfaceDirection.U, 1E-06);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(success, Is.True);
                    Assert.That(newSurf, Is.Not.Null);
                }
                foreach (var su in uSamples)
                {
                    foreach (var sv in vSamples)
                    {
                        var pos_original = surf.GetPos(su, sv);
                        var pos_new = newSurf!.GetPos(su, sv);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(1));// looser tolerance because sphere shape approximation
                            Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(1));
                            Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(1));
                        }
                    }
                }
                surf = newSurf!;
            }
        }

        [Test]
        public void KnotOperator_RemoveKnot_Cylinder()
        {
            double R = 5.0;
            var surf = PrimitiveFactory.CreateCylinder(R, 10.0).First();

            //remove knots
            double[] knotsToRemove = [ 0.5,0.75 ];
            double[] uSamples = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
            double[] vSamples = [0.0, 0.3, 0.5, 0.7, 1.0];
            foreach (var u in knotsToRemove)
            {
                var (success, newSurf) = KnotOperator.RemoveKnot(surf, u, SurfaceDirection.V, 1E-06);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(success, Is.True);
                    Assert.That(newSurf, Is.Not.Null);
                }
                foreach (var su in uSamples)
                {
                    foreach (var sv in vSamples)
                    {
                        var pos_original = surf.GetPos(su, sv);
                        var pos_new = newSurf!.GetPos(su, sv);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(1));// looser tolerance because cylinder shape approximation
                            Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(1));
                            Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(1));
                        }
                    }
                }
                surf = newSurf!;
            }
        }

        [Test]
        public void RefineKnot_Surface_UDirection()
        {
            // Create a simple planar surface
            Vector3Double p0 = new Vector3Double(0, 0, 0);
            Vector3Double p1 = new Vector3Double(10, 0, 0);
            Vector3Double p2 = new Vector3Double(0, 10, 0);
            Vector3Double p3 = new Vector3Double(10, 10, 0);

            var surf = PrimitiveFactory.CreateFace(p0, p1, p2, p3);
            int origKnotCountU = surf.KnotVectorU.Knots.Length;
            int origKnotCountV = surf.KnotVectorV.Knots.Length;
            int origCPCountU = surf.ControlPoints.Length;
            int origCPCountV = surf.ControlPoints[0].Length;

            // Refine in U direction
            double[] refineKnots = [0.25, 0.5, 0.75];
            var refinedSurf = KnotOperator.RefineKnot(surf, refineKnots, SurfaceDirection.U);

            // Check knot vector length increased in U direction
            using (Assert.EnterMultipleScope())
            {
                Assert.That(refinedSurf.KnotVectorU.Knots, Has.Length.EqualTo(origKnotCountU + 3));
                Assert.That(refinedSurf.KnotVectorV.Knots, Has.Length.EqualTo(origKnotCountV)); // V unchanged
                Assert.That(refinedSurf.ControlPoints, Has.Length.EqualTo(origCPCountU + 3));
                Assert.That(refinedSurf.ControlPoints[0], Has.Length.EqualTo(origCPCountV)); // V unchanged
            }

            // Shape preservation: sample points
            double[] uSamples = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
            double[] vSamples = [0.0, 0.3, 0.5, 0.7, 1.0];

            foreach (var u in uSamples)
            {
                foreach (var v in vSamples)
                {
                    var pos0 = surf.GetPos(u, v);
                    var pos1 = refinedSurf.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos0.X, Is.EqualTo(pos1.X).Within(1e-8));
                        Assert.That(pos0.Y, Is.EqualTo(pos1.Y).Within(1e-8));
                        Assert.That(pos0.Z, Is.EqualTo(pos1.Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void RefineKnot_Surface_VDirection()
        {
            // Create a simple planar surface
            Vector3Double p0 = new Vector3Double(0, 0, 0);
            Vector3Double p1 = new Vector3Double(10, 0, 0);
            Vector3Double p2 = new Vector3Double(0, 10, 0);
            Vector3Double p3 = new Vector3Double(10, 10, 0);

            var surf = PrimitiveFactory.CreateFace(p0, p1, p2, p3);
            int origKnotCountU = surf.KnotVectorU.Knots.Length;
            int origKnotCountV = surf.KnotVectorV.Knots.Length;
            int origCPCountU = surf.ControlPoints.Length;
            int origCPCountV = surf.ControlPoints[0].Length;

            // Refine in V direction
            double[] refineKnots = [0.25, 0.5, 0.75];
            var refinedSurf = KnotOperator.RefineKnot(surf, refineKnots, SurfaceDirection.V);

            // Check knot vector length increased in V direction
            using (Assert.EnterMultipleScope())
            {
                Assert.That(refinedSurf.KnotVectorU.Knots, Has.Length.EqualTo(origKnotCountU)); // U unchanged
                Assert.That(refinedSurf.KnotVectorV.Knots, Has.Length.GreaterThan(origKnotCountV));
                Assert.That(refinedSurf.ControlPoints, Has.Length.EqualTo(origCPCountU)); // U unchanged
                Assert.That(refinedSurf.ControlPoints[0], Has.Length.GreaterThan(origCPCountV));
            }

            // Shape preservation: sample points
            double[] uSamples = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
            double[] vSamples = [0.0, 0.3, 0.5, 0.7, 1.0];

            foreach (var u in uSamples)
            {
                foreach (var v in vSamples)
                {
                    var pos0 = surf.GetPos(u, v);
                    var pos1 = refinedSurf.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos0.X, Is.EqualTo(pos1.X).Within(1e-8));
                        Assert.That(pos0.Y, Is.EqualTo(pos1.Y).Within(1e-8));
                        Assert.That(pos0.Z, Is.EqualTo(pos1.Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void RefineKnot_Surface_WithDuplicates()
        {
            // Create a simple planar surface
            Vector3Double p0 = new Vector3Double(0, 0, 0);
            Vector3Double p1 = new Vector3Double(10, 0, 0);
            Vector3Double p2 = new Vector3Double(0, 10, 0);
            Vector3Double p3 = new Vector3Double(10, 10, 0);

            var surf = PrimitiveFactory.CreateFace(p0, p1, p2, p3);

            // Refine with duplicates and endpoints (should be handled correctly)
            double[] refineKnots = [0.5, 0.5, 0.5, 0.0, 1.0, 0.25, 0.75];
            var refinedSurf = KnotOperator.RefineKnot(surf, refineKnots, SurfaceDirection.U);

            // Shape preservation
            double[] uSamples = [0.0, 0.25, 0.5, 0.75, 1.0];
            double[] vSamples = [0.0, 0.5, 1.0];

            foreach (var u in uSamples)
            {
                foreach (var v in vSamples)
                {
                    var pos0 = surf.GetPos(u, v);
                    var pos1 = refinedSurf.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos0.X, Is.EqualTo(pos1.X).Within(1e-8));
                        Assert.That(pos0.Y, Is.EqualTo(pos1.Y).Within(1e-8));
                        Assert.That(pos0.Z, Is.EqualTo(pos1.Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void RefineKnot_Surface_Sphere()
        {
            var surf = PrimitiveFactory.CreateSphere(5.0);

            // Refine knots in U and V directions
            double[] refineKnotsU = [0.1, 0.3, 0.5, 0.7, 0.9];
            double[] refineKnotsV = [0.2, 0.4, 0.6, 0.8];
            var refinedU = KnotOperator.RefineKnot(surf, refineKnotsU, SurfaceDirection.U);
            var refinedUV = KnotOperator.RefineKnot(refinedU, refineKnotsV, SurfaceDirection.V);


            // Shape preservation
            double[] uSamples = [0.0, 0.25, 0.5, 0.75, 1.0];
            double[] vSamples = [0.0, 0.5, 1.0];

            foreach (var u in uSamples)
            {
                foreach (var v in vSamples)
                {
                    var pos0 = surf.GetPos(u, v);
                    var pos1 = refinedUV.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos0.X, Is.EqualTo(pos1.X).Within(1e-8));
                        Assert.That(pos0.Y, Is.EqualTo(pos1.Y).Within(1e-8));
                        Assert.That(pos0.Z, Is.EqualTo(pos1.Z).Within(1e-8));
                    }
                }
            }

        }

        static async Task TestOutIGES(List<NurbsSurface> geometries, string filePath = "testknot.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            await IGESExporter.ExportAsync(geometries, stream);
        }

        static async Task TestOutIGES(List<NurbsCurve> geometry, string filePath = "testknot.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            await IGESExporter.ExportAsync(geometry, stream);
        }
    }
}
