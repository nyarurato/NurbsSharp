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
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(R ,  0, 0, 1),
                new ControlPoint(R ,  R, 0, 0.70710678),
                new ControlPoint(0 ,  R, 0, 1),
                new ControlPoint(-R,  R, 0, 0.70710678),
                new ControlPoint(-R,  0, 0, 1),
                new ControlPoint(-R, -R, 0, 0.70710678),
                new ControlPoint(0 , -R, 0, 1),
                new ControlPoint(R , -R, 0, 0.70710678),
                new ControlPoint(R ,  0, 0, 1),

            };
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
                Assert.That(new_knots.Length, Is.EqualTo(knots.Length + insert_count + 1));
                Assert.That(new_cps.Length, Is.EqualTo(controlPoints.Length + insert_count + 1));
                foreach (var s in samplePoints)
                {
                    var pos_original = curve.GetPos(s);
                    var pos_new = new_curve.GetPos(s);
                    Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                    Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                    Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                }
                insert_count++;
            }
        }

        [Test]
        public void KnotOperator_Insert_TestB()
        {
            // Line
            int degree = 1;
            double[] knots = { 0, 0, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0 ,  0, 0, 1),
                new ControlPoint(10 , 0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = { 0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1 };
            double[] insert_u = { 0.5, 0.3, 0.8, 0.111, 0.2, 0.9999 };
            double[] new_knots = knots;
            ControlPoint[] new_cps = controlPoints;
            int insert_count = 0;
            foreach (var u in insert_u)
            {
                (new_knots, new_cps) = KnotOperator.InsertKnot(degree, new_knots, new_cps, u, 1);
                var new_curve = new NurbsCurve(degree, new KnotVector(new_knots, degree), new_cps);
                Assert.That(new_knots.Length, Is.EqualTo(knots.Length + insert_count + 1));
                Assert.That(new_cps.Length, Is.EqualTo(controlPoints.Length + insert_count + 1));
                foreach (var s in samplePoints)
                {
                    var pos_original = curve.GetPos(s);
                    var pos_new = new_curve.GetPos(s);
                    Assert.That(pos_new.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                    Assert.That(pos_new.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                    Assert.That(pos_new.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                }
                insert_count++;
            }
        }


        [Test]
        public void SplitOperator_SplitCurve_TestA()
        {
            // Circle
            int R = 2;
            int degree = 2;
            double[] knots = { 0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(R ,  0, 0, 1),
                new ControlPoint(R ,  R, 0, 0.70710678),
                new ControlPoint(0 ,  R, 0, 1),
                new ControlPoint(-R,  R, 0, 0.70710678),
                new ControlPoint(-R,  0, 0, 1),
                new ControlPoint(-R, -R, 0, 0.70710678),
                new ControlPoint(0 , -R, 0, 1),
                new ControlPoint(R , -R, 0, 0.70710678),
                new ControlPoint(R ,  0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] split_us = { 0.3, 0.5, 0.75 };
            double[] sample_points = { 0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1 };

            foreach (var u in split_us)
            {
                var (leftCurve, rightCurve) = SplitOperator.SplitCurve(curve, u);
                
                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    var pos_original = curve.GetPos(s);

                    Vector3Double pos;
                    if (s > u)//check s belong left or righ
                        pos = rightCurve.GetPos(s);
                    else
                        pos = leftCurve.GetPos(s);
                    Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                    Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                    Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                    
                }
               
            }
        }


        [Test]
        [Ignore("not implemented yet")]

        public void DegreeOperator_ElevateDegree_TestB()
        {
            // Quadratic curve
            int degree = 2;
            double[] knots = { 0, 0, 0, 1, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(5, 10, 0, 1),
                new ControlPoint(10, 0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = { 0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0 };

            // Elevate degree once
            var elevatedCurve = DegreeOperator.ElevateDegree(curve, 1);
            Assert.That(elevatedCurve.Degree, Is.EqualTo(3));

            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_elevated = elevatedCurve.GetPos(u);
                Assert.That(pos_elevated.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                Assert.That(pos_elevated.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                Assert.That(pos_elevated.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
            }
        }

        [Test]
        [Ignore("not implemented yet")]
        public void DegreeOperator_ReduceDegree_TestA()
        {
            
            // Start with a line (degree 1), elevate it, then reduce it back
            int degree = 1;
            double[] knots = { 0, 0, 1, 1 };
            ControlPoint[] controlPoints = {
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(10, 0, 0, 1),
            };
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            double[] samplePoints = { 0, 0.1, 0.2, 0.5, 0.7, 0.9, 1.0 };

            // Elevate degree
            var elevatedCurve = DegreeOperator.ElevateDegree(curve, 1);
            Assert.That(elevatedCurve.Degree, Is.EqualTo(2));

            // Reduce degree back
            var reducedCurve = DegreeOperator.ReduceDegree(elevatedCurve, 1, 1e-3);
            Assert.That(reducedCurve, Is.Not.Null);
            Assert.That(reducedCurve.Degree, Is.EqualTo(1));

            // Check that the reduced curve approximates the original
            foreach (var u in samplePoints)
            {
                var pos_original = curve.GetPos(u);
                var pos_reduced = reducedCurve.GetPos(u);
                Assert.That(pos_reduced.X, Is.EqualTo(pos_original.X).Within(0.1));
                Assert.That(pos_reduced.Y, Is.EqualTo(pos_original.Y).Within(0.1));
                Assert.That(pos_reduced.Z, Is.EqualTo(pos_original.Z).Within(0.1));
            }
        }
    }
}
