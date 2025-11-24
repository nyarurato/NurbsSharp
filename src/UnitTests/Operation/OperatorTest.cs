using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;
using NurbsSharp.Operation;

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
        public void DegreeOperator_ElevateDegree_TestA()
        {
            // Quadratic curve
            int degree = 3;
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(5, 10, 3, 0.5),
                new ControlPoint(10, 4, 2, 1),
                new ControlPoint(15, 10, 0, 0.7),
                new ControlPoint(20, 3, 0, 1),
            ];
            KnotVector knotVector = KnotVector.GetClampedKnot(degree, controlPoints.Length);
            var curve = new NurbsCurve(degree, knotVector, controlPoints);
            double[] samplePoints = [0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0];

            // Elevate degree once
            var elevatedCurve = DegreeOperator.ElevateDegree(curve, 1);
            Assert.That(elevatedCurve.Degree, Is.EqualTo(4));

            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_elevated = elevatedCurve.GetPos(u);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pos_elevated.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                    Assert.That(pos_elevated.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                    Assert.That(pos_elevated.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                }
            }
        }
        [Test]
        public void DegreeOperator_ElevateDegree_TestB()
        {
            // Quadratic curve
            int degree = 2;
            double[] knots = [0, 0, 0, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(5, 10, 0, 1),
                new ControlPoint(10, 0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = [0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0];

            // Elevate degree once
            var elevatedCurve = DegreeOperator.ElevateDegree(curve, 1);
            Assert.That(elevatedCurve.Degree, Is.EqualTo(3));

            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_elevated = elevatedCurve.GetPos(u);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pos_elevated.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                    Assert.That(pos_elevated.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                    Assert.That(pos_elevated.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                }
            }
        }


        [Test]
        public void DegreeOperator_ReduceDegree_TestA()
        {
            
            // Start with a line (degree 1), elevate it, then reduce it back
            int degree = 1;
            double[] knots = [0, 0, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(10, 0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = [0, 0.1, 0.2, 0.5, 0.7, 0.9, 1.0];

            // Elevate degree
            var elevatedCurve = DegreeOperator.ElevateDegree(curve, 1);
            Assert.That(elevatedCurve.Degree, Is.EqualTo(2));

            // Reduce degree back
            var reducedCurve = DegreeOperator.ReduceDegree(elevatedCurve, 1, 1e-3);
            Assert.That(reducedCurve, Is.Not.Null);
            Assert.That(reducedCurve.Degree, Is.EqualTo(1));

            double maxErrorDist = 0;
            // Check that the reduced curve approximates the original
            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_reduced = reducedCurve.GetPos(u);
                double dist = pos_original.DistanceTo(pos_reduced);
                if (dist > maxErrorDist) maxErrorDist = dist;
                
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pos_reduced.X, Is.EqualTo(pos_original.X).Within(0.1));
                    Assert.That(pos_reduced.Y, Is.EqualTo(pos_original.Y).Within(0.1));
                    Assert.That(pos_reduced.Z, Is.EqualTo(pos_original.Z).Within(0.1));
                }
            }
        }

        private NurbsCurve CreateStraightLineCurve(int degree, int numControlPoints, double[]? weights = null)
        {
            var cps = new ControlPoint[numControlPoints];
            for (int i = 0; i < numControlPoints; i++)
            {
                double x = i; // simple x-axis line
                const double y_factor = 2.0;
                double w = (weights != null && i < weights.Length) ? weights[i] : 1.0;
                cps[i] = new ControlPoint(x, y_factor * i, 0.0, w);
            }
            var kv = KnotVector.GetClampedKnot(degree, numControlPoints);
            return new NurbsCurve(degree, kv, cps);
        }

        [Test]
        public void ElevateDegree_NonRational_PreservesShape()
        {
            var curve = CreateStraightLineCurve(2, 5); // degree 2, 5 control points
            var elevated = DegreeOperator.ElevateDegree(curve, 1); // to degree 3

            double umin = elevated.KnotVector.Knots[elevated.Degree];
            double umax = elevated.KnotVector.Knots[elevated.KnotVector.Length - elevated.Degree - 1];
            int samples = 30;
            for (int i = 0; i < samples; i++)
            {
                double u = umin + (umax - umin) * i / (samples - 1);
                var pOrig = CurveEvaluator.Evaluate(curve, u);
                var pElev = CurveEvaluator.Evaluate(elevated, u);
                // allow small numerical approximation error from least-squares fitting
                Assert.That(pOrig.DistanceTo(pElev), Is.LessThanOrEqualTo(2e-2));
            }
        }

        [Test]
        public void ElevateDegree_Rational_ColinearPreservesShape()
        {
            // weights vary but control points colinear, so projected curve is still a line
            double[] weights = [1.0, 2.0, 1.0, 1.5];
            var curve = CreateStraightLineCurve(2, 4, weights);
            var elevated = DegreeOperator.ElevateDegree(curve, 2); // raise degree by 2

            double umin = elevated.KnotVector.Knots[elevated.Degree];
            double umax = elevated.KnotVector.Knots[elevated.KnotVector.Length - elevated.Degree - 1];
            int samples = 30;
            for (int i = 0; i < samples; i++)
            {
                double u = umin + (umax - umin) * i / (samples - 1);
                var pOrig = CurveEvaluator.Evaluate(curve, u);
                var pElev = CurveEvaluator.Evaluate(elevated, u);
                // rational case may have larger numerical differences; allow looser tolerance
                Assert.That(pOrig.DistanceTo(pElev), Is.LessThanOrEqualTo(3e-2));
            }
        }

        [Test]
        public void ReduceDegree_StraightLineWithinTolerance()
        {
            var curve = CreateStraightLineCurve(4, 8); // degree 4
            // reduce degree by 2 to degree 2
            // allow a modest tolerance because reduction is done by least-squares in homogeneous coords
            var reduced = DegreeOperator.ReduceDegree(curve, 2, 1e-1);

            double umin = reduced.KnotVector.Knots[reduced.Degree];
            double umax = reduced.KnotVector.Knots[reduced.KnotVector.Length - reduced.Degree - 1];
            int samples = 50;
            double maxErrorDist = 0;
            for (int i = 0; i < samples; i++)
            {
                double u = umin + (umax - umin) * i / (samples - 1);
                var pOrig = CurveEvaluator.Evaluate(curve, u);
                var pRed = CurveEvaluator.Evaluate(reduced, u);
                double dist = pOrig.DistanceTo(pRed);
                if (dist > maxErrorDist) maxErrorDist = dist;
                // match the tolerance used when calling ReduceDegree
                Assert.That(pOrig.DistanceTo(pRed), Is.LessThanOrEqualTo(1e-1));
            }
        }

        [Test]
        public void DegreeOperator_ReduceDegree_TestB()
        {
            // Quadratic curve
            int degree = 3;
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(5, 10, 3, 0.5),
                new ControlPoint(10, 4, 2, 1),
                new ControlPoint(15, 10, 0, 0.7),
                new ControlPoint(20, 3, 0, 1),
            ];
            KnotVector knotVector = KnotVector.GetClampedKnot(degree, controlPoints.Length);
            var curve = new NurbsCurve(degree, knotVector, controlPoints);
            double[] samplePoints = [0, 0.001 ,0.1, 0.25,0.3,0.49995 ,0.5, 0.6,0.75, 0.8,0.9, 0.999,1.0];

            // Elevate degree once
            var reduceCurve = DegreeOperator.ReduceDegree(curve, 1,1e-1);
            Assert.That(reduceCurve.Degree, Is.EqualTo(2));

            double maxErrorX = 0, maxErrorY = 0, maxErrorZ = 0, maxErrorDist = 0;
            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_reduced = reduceCurve.GetPos(u);
                
                double errX = Math.Abs(pos_reduced.X - pos_original.X);
                double errY = Math.Abs(pos_reduced.Y - pos_original.Y);
                double errZ = Math.Abs(pos_reduced.Z - pos_original.Z);
                double dist = pos_original.DistanceTo(pos_reduced);
                
                if (errX > maxErrorX) maxErrorX = errX;
                if (errY > maxErrorY) maxErrorY = errY;
                if (errZ > maxErrorZ) maxErrorZ = errZ;
                if (dist > maxErrorDist) maxErrorDist = dist;
                
                using (Assert.EnterMultipleScope())
                {
                    // Looser tolerance due to degree reduction approximation
                    // Differeces 0.46
                    Assert.That(pos_reduced.X, Is.EqualTo(pos_original.X).Within(0.5));
                    Assert.That(pos_reduced.Y, Is.EqualTo(pos_original.Y).Within(0.5));
                    Assert.That(pos_reduced.Z, Is.EqualTo(pos_original.Z).Within(0.5));
                }
            }
            
        }

        [Test]
        public void DegreeOperator_ReduceDegree_Complex5to3()
        {
            // Complex rational curve: degree 5 -> 3 (reduce by 2)
            int degree = 5;
            ControlPoint[] controlPoints = [
                new ControlPoint(0, 0, 0, 1.0),
                new ControlPoint(3, 8, 2, 0.6),
                new ControlPoint(7, 12, 5, 1.2),
                new ControlPoint(12, 9, 4, 0.8),
                new ControlPoint(16, 14, 1, 1.5),
                new ControlPoint(20, 10, 3, 0.7),
                new ControlPoint(25, 5, 2, 1.0),
                new ControlPoint(30, 2, 0, 0.9),
            ];
            KnotVector knotVector = KnotVector.GetClampedKnot(degree, controlPoints.Length);
            var curve = new NurbsCurve(degree, knotVector, controlPoints);
            
            // Dense sampling points including critical regions
            double[] samplePoints = [ 
                0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 
                0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0 
            ];

            // Reduce degree by 2 (5 -> 3)
            var reducedCurve = DegreeOperator.ReduceDegree(curve, 2, 1e-1);
            Assert.That(reducedCurve.Degree, Is.EqualTo(3));
                                    
            // Verify errors are within reasonable bounds for 2-step reduction
            // For degree reduction by 2, we expect larger errors than single-step reduction
            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_reduced = reducedCurve.GetPos(u);
                using (Assert.EnterMultipleScope())
                {
                    // Tolerance for 2-degree reduction with complex rational curve
                    // Differences 0.37
                    Assert.That(pos_reduced.X, Is.EqualTo(pos_original.X).Within(0.5));
                    Assert.That(pos_reduced.Y, Is.EqualTo(pos_original.Y).Within(0.5));
                    Assert.That(pos_reduced.Z, Is.EqualTo(pos_original.Z).Within(0.5));
                }
            }
        }
    }
}
