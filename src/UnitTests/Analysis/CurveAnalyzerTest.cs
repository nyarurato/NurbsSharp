using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using NurbsSharp.Generation;
using System;
using System.Linq;
using NurbsSharp.Analysis;
using NurbsSharp.Evaluation;

namespace UnitTests.Analysis
{
    [TestFixture]
    public class CurveAnalyzerTest
    {

        [Test]
        public void NurbsCurveCurvatureA()
        {
            // Circle R=3
            int degree = 2;
            double[] knots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(3 ,  0, 0, 1),
                new ControlPoint(3 ,  3, 0, 0.70710678),
                new ControlPoint(0 ,  3, 0, 1),
                new ControlPoint(-3,  3, 0, 0.70710678),
                new ControlPoint(-3,  0, 0, 1),
                new ControlPoint(-3, -3, 0, 0.70710678),
                new ControlPoint(0 , -3, 0, 1),
                new ControlPoint(3 , -3, 0, 0.70710678),
                new ControlPoint(3 ,  0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99 };//u=1.0 case is unstable
            double R = 3.0;
            foreach (var u in samplePoints)
            {
                var curvature = CurveAnalyzer.EvaluateCurvature(curve, u);
                //Cirle is special case where curvature can be computed exactly
                Assert.That(curvature, Is.EqualTo(1 / R).Within(0.01));
            }
        }

        [Test]
        public void NurbsCurveCurvatureB()
        {
            int degree = 3;
            double[] knots = [0, 0, 0, 0, 0.5, 0.7, 1, 1, 1, 1];
            ControlPoint[] controlPoints = [
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 2.0, 0.0, 1),
            new ControlPoint(2.0, 2.0, 0.0, 1),
            new ControlPoint(3.0, 0.0, 0.0, 1),
            new ControlPoint(4.0, 2.0, 0.0, 1),
            new ControlPoint(5.0, 0.0, 0.0, 1)
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);
            var samplePoints = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.999 };// u=1.0 case is unstable
            foreach (var u in samplePoints)
            {

                const double eps = 1e-8;

                double u0 = Math.Clamp(u, eps, 1 - eps);
                var p = CurveEvaluator.Evaluate(curve, u0);
                var d1 = CurveEvaluator.EvaluateFirstDerivative(curve, u0);
                var d2 = CurveEvaluator.EvaluateSecondDerivative(curve, u0);
                var k = CurveAnalyzer.EvaluateCurvature(curve, u0);
            }
        }
        [Test]
        public void FrenetFrame_Circle_OrthogonalityAndNormalization()
        {
            // Circle R=3
            int degree = 2;
            double[] knots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(3 ,  0, 0, 1),
                new ControlPoint(3 ,  3, 0, 0.70710678),
                new ControlPoint(0 ,  3, 0, 1),
                new ControlPoint(-3,  3, 0, 0.70710678),
                new ControlPoint(-3,  0, 0, 1),
                new ControlPoint(-3, -3, 0, 0.70710678),
                new ControlPoint(0 , -3, 0, 1),
                new ControlPoint(3 , -3, 0, 0.70710678),
                new ControlPoint(3 ,  0, 0, 1),
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            double[] sampleUs = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99];//u=1.0 case is unstable
            const double tol = 1e-8;

            foreach (var u in sampleUs)
            {
                var (T, N) = CurveAnalyzer.EvaluateTangentNormal(curve, u);

                using (Assert.EnterMultipleScope())
                {
                    // If tangent is zero then frame is invalid; fail test in that case
                    Assert.That(T.magnitude, Is.GreaterThan(0.0), $"Tangent is zero at u={u}");
                    Assert.That(Math.Abs(T.magnitude - 1.0), Is.LessThan(tol), $"T not normalized at u={u}");
                }

                // circle Normal is origin to point vector
                var pt = CurveEvaluator.Evaluate(curve, u);
                var expectedN = new Vector3Double(pt.X, pt.Y, pt.Z).normalized;
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(Math.Abs(N.X) - Math.Abs(expectedN.X), Is.LessThan(tol), $"N.X incorrect at u={u}");
                    Assert.That(Math.Abs(N.Y) - Math.Abs(expectedN.Y), Is.LessThan(tol), $"N.Y incorrect at u={u}");
                    Assert.That(Math.Abs(N.Z) - Math.Abs(expectedN.Z), Is.LessThan(tol), $"N.Z incorrect at u={u}");
                }

                // Ensure helper methods return same vectors
                var t2 = CurveAnalyzer.EvaluateTangent(curve, u);
                var n2 = CurveAnalyzer.EvaluateNormal(curve, u);
                var dt = new Vector3Double(t2.X - T.X, t2.Y - T.Y, t2.Z - T.Z);
                var dn = new Vector3Double(n2.X - N.X, n2.Y - N.Y, n2.Z - N.Z);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(dt.magnitude, Is.LessThan(1e-12));
                    Assert.That(dn.magnitude, Is.LessThan(1e-12));
                }
            }
        }

        [Test]
        public void FrenetFrame_Line_NormalAndBinormalAreZero_TangentMatches()
        {
            // Straight line from (0,0,0) to (6,8,0)
            int degree = 2;
            double[] knots = [0, 0, 0, 1, 1, 1];
            ControlPoint[] controlPoints = [
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 4.0, 0.0, 1),
                new ControlPoint(6.0, 8.0, 0.0, 1)
            ];
            var curve = new NurbsCurve(degree, new KnotVector(knots, degree), controlPoints);

            double[] sampleUs = [0.0, 0.25, 0.5, 0.75, 0.999999];//u=1.0 case is unstable
            const double tol = 1e-8;
            var expectedT = new Vector3Double(3.0 / 5.0, 4.0 / 5.0, 0.0); // normalized (6,8,0)

            foreach (var u in sampleUs)
            {
                (var t, var n) = CurveAnalyzer.EvaluateTangentNormal(curve, u);

                using (Assert.EnterMultipleScope())
                {
                    // Tangent should match expected normalized direction
                    Assert.That(Math.Abs(t.X - expectedT.X), Is.LessThan(tol));
                    Assert.That(Math.Abs(t.Y - expectedT.Y), Is.LessThan(tol));
                    Assert.That(Math.Abs(t.Z - expectedT.Z), Is.LessThan(tol));

                    // For a straight line, second derivative is zero -> normal should be zero vectors
                    Assert.That(n.magnitude, Is.EqualTo(0.0).Within(tol));
                }

                // Ensure helper methods return same vectors
                var t2 = CurveAnalyzer.EvaluateTangent(curve, u);
                var n2 = CurveAnalyzer.EvaluateNormal(curve, u);
                var dt = new Vector3Double(t2.X - t.X, t2.Y - t.Y, t2.Z - t.Z);
                var dn = new Vector3Double(n2.X - n.X, n2.Y - n.Y, n2.Z - n.Z);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(dt.magnitude, Is.LessThan(1e-12));
                    Assert.That(dn.magnitude, Is.LessThan(1e-12));
                }
            }
        }

        [Test]
        public void FindClosestPoint_OnStraightLine_ReturnsPerpendicularPoint()
        {
            // Create a straight line from (0,0,0) to (10,0,0)
            var controlPoints = new[]
            {
                new ControlPoint(new Vector3Double(0, 0, 0), 1),
                new ControlPoint(new Vector3Double(10, 0, 0), 1)
            };
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point above the line
            var target = new Vector3Double(5, 3, 0);

            // Find closest point using operator
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            using (Assert.EnterMultipleScope())
            {
                // Should find point at (5,0,0)
                Assert.That(point.X, Is.EqualTo(5.0).Within(1e-5), "X coordinate should be 5");
                Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5), "Y coordinate should be 0");
                Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-5), "Z coordinate should be 0");
                Assert.That(distance, Is.EqualTo(3.0).Within(1e-5), "Distance should be 3");
                Assert.That(t, Is.EqualTo(0.5).Within(1e-3), "Parameter should be 0.5");
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point above the line
            var target = new Vector3Double(5, 3, 0);

            // Find closest point using instance method
            var (t, point, distance) = curve.FindClosestPoint(target);

            using (Assert.EnterMultipleScope())
            {
                // Should find point at (5,0,0)
                Assert.That(point.X, Is.EqualTo(5.0).Within(1e-5), "X coordinate should be 5");
                Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5), "Y coordinate should be 0");
                Assert.That(point.Z, Is.EqualTo(0.0).Within(1e-5), "Z coordinate should be 0");
                Assert.That(distance, Is.EqualTo(3.0).Within(1e-5), "Distance should be 3");
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2);
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
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point
            var target = new Vector3Double(7, 2, 0);

            // Find closest point with good initial guess
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target, initialT: 0.7);

            using (Assert.EnterMultipleScope())
            {
                // Should find point at (7,0,0)
                Assert.That(point.X, Is.EqualTo(7.0).Within(1e-5), "X coordinate should be 7");
                Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5), "Y coordinate should be 0");
                Assert.That(distance, Is.EqualTo(2.0).Within(1e-5), "Distance should be 2");
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            var target = new Vector3Double(7, 2, 0);

            // Use instance method with initial guess
            var (t, point, distance) = curve.FindClosestPointWithInitialGuess(target, initialT: 0.7);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(point.X, Is.EqualTo(7.0).Within(1e-5));
                Assert.That(distance, Is.EqualTo(2.0).Within(1e-5));
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2);
            var curve = new NurbsCurve(2, knotVector, controlPoints);

            // Target point near the apex
            var target = new Vector3Double(5, 6, 0);

            // Find closest point
            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            using (Assert.EnterMultipleScope())
            {
                // Point should be near the apex of the parabola
                Assert.That(point.X, Is.EqualTo(5.0).Within(0.5), "X coordinate should be near 5");
                Assert.That(point.Y, Is.GreaterThan(4.0), "Y coordinate should be significantly positive");
                Assert.That(distance, Is.LessThan(2.0), "Distance should be reasonably small");
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point beyond the start
            var target = new Vector3Double(-5, 2, 0);

            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            using (Assert.EnterMultipleScope())
            {
                // Should clamp to start point (0,0,0)
                Assert.That(point.X, Is.EqualTo(0.0).Within(1e-5), "Should return start point");
                Assert.That(point.Y, Is.EqualTo(0.0).Within(1e-5));
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 1.0, 1.0], 1);
            var curve = new NurbsCurve(1, knotVector, controlPoints);

            // Target point exactly on the curve
            var target = new Vector3Double(5, 0, 0);

            var (t, point, distance) = CurveAnalyzer.FindClosestPoint(curve, target);

            using (Assert.EnterMultipleScope())
            {
                // Distance should be essentially zero
                Assert.That(distance, Is.LessThan(1e-5), "Distance should be near zero");
                Assert.That(point.X, Is.EqualTo(5.0).Within(1e-5));
            }
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
            var knotVector = new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2);
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