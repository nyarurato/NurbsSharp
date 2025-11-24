using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.IO.IGES;
using NurbsSharp.Operation;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace UnitTests.Operation
{
    [TestFixture]
    public class JoinOperatorTest
    {
        [Test]
        public void JoinLinearCurves()
        {
            // Curve 1: (0,0,0) -> (1,0,0)
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 0, 0)
            };
            var kv1 = KnotVector.GetClampedKnot(1, 2); // {0,0,1,1}
            var c1 = new NurbsCurve(1, kv1, cp1);

            // Curve 2: (1,0,0) -> (2,0,0)
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(1, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var kv2 = KnotVector.GetClampedKnot(1, 2); // {0,0,1,1}
            var c2 = new NurbsCurve(1, kv2, cp2);

            var joined = JoinOperator.JoinCurves(c1, c2);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(joined.Degree, Is.EqualTo(1));
                Assert.That(joined.ControlPoints, Has.Length.EqualTo(3));
                Assert.That(joined.ControlPoints[0].Position.X, Is.Zero);
                Assert.That(joined.ControlPoints[1].Position.X, Is.EqualTo(1));
                Assert.That(joined.ControlPoints[2].Position.X, Is.EqualTo(2));
            }

            // Expected knots: 0,0, 1, 2,2
            using (Assert.EnterMultipleScope())
            {
                Assert.That(joined.KnotVector.Knots, Has.Length.EqualTo(5));
                Assert.That(joined.KnotVector.Knots[0], Is.Zero);
                Assert.That(joined.KnotVector.Knots[1], Is.Zero);
                Assert.That(joined.KnotVector.Knots[2], Is.EqualTo(1));
                Assert.That(joined.KnotVector.Knots[3], Is.EqualTo(2)); // shifted by 1
                Assert.That(joined.KnotVector.Knots[4], Is.EqualTo(2));
            }

            Assert.That(joined.GetLength(), Is.EqualTo(2.0).Within(1e-6));

            double[] sampler_point = [0.0,0.001, 0.5, 0.99,1.0,1.01,1.5, 1.999,2.0];
            foreach (var u in sampler_point)
            {
                Vector3Double estimated_u;
                if(u <= 1.0)
                {
                    estimated_u = c1.GetPos(u); // First curve
                }
                else
                {
                    estimated_u = c2.GetPos(u - 1.0); // Second curve
                }
                Vector3Double joined_u = joined.GetPos(u);
                Assert.That(joined_u,Is.EqualTo(estimated_u));
            }

        }

        [Test]
        public void JoinQuadraticCurves()
        {
            // Curve 1: (0,0,0) -> (1,1,0) -> (2,0,0)
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 0),
                new ControlPoint(2, 0, 0)
            };
            var kv1 = KnotVector.GetClampedKnot(2, 3); // {0,0,0,1,1,1}
            var c1 = new NurbsCurve(2, kv1, cp1);

            // Curve 2: (2,0,0) -> (3,-1,0) -> (4,0,0)
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(2, 0, 0),
                new ControlPoint(3, -1, 0),
                new ControlPoint(4, 0, 0)
            };
            var kv2 = KnotVector.GetClampedKnot(2, 3); // {0,0,0,1,1,1}
            var c2 = new NurbsCurve(2, kv2, cp2);

            var joined = JoinOperator.JoinCurves(c1, c2);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(joined.Degree, Is.EqualTo(2));
                // 3 + 3 - 1 = 5 CPs
                Assert.That(joined.ControlPoints, Has.Length.EqualTo(5));
            }

            // Expected knots: 0,0,0, 1,1, 2,2,2
            // Length: 5 + 2 + 1 = 8
            using (Assert.EnterMultipleScope())
            {
                Assert.That(joined.KnotVector.Knots, Has.Length.EqualTo(8));
                Assert.That(joined.KnotVector.Knots[3], Is.EqualTo(1));
                Assert.That(joined.KnotVector.Knots[4], Is.EqualTo(1));
                Assert.That(joined.KnotVector.Knots[5], Is.EqualTo(2));
            }

            double[] sampler_point = [0.0, 0.001, 0.5, 0.99, 1.0, 1.01, 1.5, 1.999, 2.0];
            foreach (var u in sampler_point)
            {
                Vector3Double estimated_u;
                if (u <= 1.0)
                {
                    estimated_u = c1.GetPos(u); // First curve
                }
                else
                {
                    estimated_u = c2.GetPos(u - 1.0); // Second curve
                }
                Vector3Double joined_u = joined.GetPos(u);
                Assert.That(joined_u, Is.EqualTo(estimated_u));
            }
        }

        [Test]
        public void JoinDifferentDegrees()
        {
            // Curve 1: Degree 1
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 0, 0)
            };
            var kv1 = KnotVector.GetClampedKnot(1, 2);
            var c1 = new NurbsCurve(1, kv1, cp1);

            // Curve 2: Degree 2
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(1, 0, 0),
                new ControlPoint(2, 1, 0),
                new ControlPoint(3, 0, 0)
            };
            var kv2 = KnotVector.GetClampedKnot(2, 3);
            var c2 = new NurbsCurve(2, kv2, cp2);

            var joined = JoinOperator.JoinCurves(c1, c2);

            using (Assert.EnterMultipleScope())
            {
                // Should be elevated to degree 2
                Assert.That(joined.Degree, Is.EqualTo(2));

                // Curve 1 elevated to degree 2 will have 3 CPs (0,0,0), (0.5,0,0), (1,0,0) Or similar.
                // Degree elevation of linear curve (0,0)->(1,0) to degree 2:
                // New CPs: (0,0), (0.5,0), (1,0)
                // Knots: 0,0,0, 1,1,1

                // Then join with Curve 2 (3 CPs)
                // Total CPs: 3 + 3 - 1 = 5
                Assert.That(joined.ControlPoints, Has.Length.EqualTo(5));
            }

            double[] sampler_point = [0.0, 0.001, 0.5, 0.99, 1.0, 1.01, 1.5, 1.999, 2.0];
            foreach (var u in sampler_point)
            {
                Vector3Double estimated_u;
                if (u <= 1.0)
                {
                    estimated_u = c1.GetPos(u); // First curve
                }
                else
                {
                    estimated_u = c2.GetPos(u - 1.0); // Second curve
                }
                Vector3Double joined_u = joined.GetPos(u);
                Assert.That(joined_u, Is.EqualTo(estimated_u));
            }
        }

        [Test]
        public void JoinDisjointCurves_ThrowsException()
        {
            var cp1 = new ControlPoint[] { new ControlPoint(0,0,0), new ControlPoint(1,0,0) };
            var c1 = new NurbsCurve(1, KnotVector.GetClampedKnot(1,2), cp1);

            var cp2 = new ControlPoint[] { new ControlPoint(2,0,0), new ControlPoint(3,0,0) };
            var c2 = new NurbsCurve(1, KnotVector.GetClampedKnot(1,2), cp2);

            Assert.Throws<InvalidOperationException>(() => JoinOperator.JoinCurves(c1, c2));
        }

        [Test]
        public void JoinCurves_DifferentWeights_ThrowsException()
        {
            // Curve 1: End point at (1,0,0) with weight 1.0
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0, 1.0),
                new ControlPoint(1, 0, 0, 1.0)
            };
            var kv1 = KnotVector.GetClampedKnot(1, 2);
            var c1 = new NurbsCurve(1, kv1, cp1);

            // Curve 2: Start point at (1,0,0) but with weight 2.0
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(1, 0, 0, 2.0), // Different weight!
                new ControlPoint(2, 0, 0, 1.0)
            };
            var kv2 = KnotVector.GetClampedKnot(1, 2);
            var c2 = new NurbsCurve(1, kv2, cp2);

            // Should throw because weights don't match
            var ex = Assert.Throws<InvalidOperationException>(() => JoinOperator.JoinCurves(c1, c2));
            Assert.That(ex.Message, Does.Contain("weight"));
        }

        [Test]
        public void JoinCurves_CustomTolerance()
        {
            // Curve 1: End point at (1,0,0)
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 0, 0)
            };
            var kv1 = KnotVector.GetClampedKnot(1, 2);
            var c1 = new NurbsCurve(1, kv1, cp1);

            // Curve 2: Start point at (1.0001,0,0) - slightly off
            var cp2 = new ControlPoint[]
            {
                new ControlPoint(1.0001, 0, 0),
                new ControlPoint(2, 0, 0)
            };
            var kv2 = KnotVector.GetClampedKnot(1, 2);
            var c2 = new NurbsCurve(1, kv2, cp2);

            // Should fail with default tolerance
            Assert.Throws<InvalidOperationException>(() => JoinOperator.JoinCurves(c1, c2));

            // Should succeed with larger tolerance
            var joined = JoinOperator.JoinCurves(c1, c2, tolerance: 1e-3);
            Assert.That(joined.ControlPoints, Has.Length.EqualTo(3));

            double[] sampler_point = [0.0, 0.001, 0.5, 0.99, 1.0, 1.01, 1.5, 1.999, 2.0];
            foreach (var u in sampler_point)
            {
                Vector3Double estimated_u;
                if (u <= 1.0)
                {
                    estimated_u = c1.GetPos(u); // First curve
                }
                else
                {
                    estimated_u = c2.GetPos(u - 1.0); // Second curve
                }
                Vector3Double joined_u = joined.GetPos(u);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(joined_u.X, Is.EqualTo(estimated_u.X).Within(1e-3));
                    Assert.That(joined_u.Y, Is.EqualTo(estimated_u.Y).Within(1e-3));
                    Assert.That(joined_u.Z, Is.EqualTo(estimated_u.Z).Within(1e-3));
                }
            }
        }

        [Test]
        public void JoinCurvesDegree4withDiffrentDegree()
        {
            var cp1 = new ControlPoint[]
            {
                new ControlPoint(0, 0, 1),
                new ControlPoint(1, 2, 2),
                new ControlPoint(2, 0, 3),
                new ControlPoint(3, -2, 4),
                new ControlPoint(4, 0, 5)
            };

            var kv1 = KnotVector.GetClampedKnot(4, 5); // Degree 4, 5 CPs
            var c1 = new NurbsCurve(4, kv1, cp1);

            var cp2 = new ControlPoint[]
            {
                new ControlPoint(4, 0, 5),
                new ControlPoint(-5, -5, -5)
            };
            var kv2 = KnotVector.GetClampedKnot(1, 2); // Degree 1, 2 CPs
            var c2 = new NurbsCurve(1, kv2, cp2);

            var joined = JoinOperator.JoinCurves(c1, c2);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(joined.Degree, Is.EqualTo(4));
                Assert.That(joined.ControlPoints, Has.Length.EqualTo(9)); // 5 + 2 + 3(elevation) - 1 = 9
            }

            double[] sampler_point = [0.0, 0.001, 0.5, 0.99, 1.0, 1.01, 1.5, 1.999, 2.0];
            foreach (var u in sampler_point)
            {
                Vector3Double estimated_u;
                if (u <= 1.0)
                {
                    estimated_u = c1.GetPos(u); // First curve
                }
                else
                {
                    estimated_u = c2.GetPos(u - 1.0); // Second curve
                }
                Vector3Double joined_u = joined.GetPos(u);
                Assert.That(joined_u, Is.EqualTo(estimated_u));
            }
            

        }

        static void TestOutIGES(List<NurbsCurve> geometries, string filePath="testcurve.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);

            var success = IGESExporter.ExportAsync(geometries, stream);
        }
    }
}
