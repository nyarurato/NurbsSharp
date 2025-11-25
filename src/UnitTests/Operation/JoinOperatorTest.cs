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

        [Test]
        public void JoinSurfaces_UDirection_BasicTest()
        {
            int degreeU = 2;
            int degreeV = 2;
            var knotsU1 = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU1 = new KnotVector(knotsU1, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var cp1 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp1[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp1[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }
            var surface1 = new NurbsSurface(degreeU, degreeV, kvU1, kvV, cp1);
            //U end V: (2,0,0),(2,1,0),(2,2,0)

            var knotsU2 = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU2 = new KnotVector(knotsU2, degreeU);

            var cp2 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp2[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp2[i][j] = new ControlPoint(i + 2, j, 0, 1);
                }
            }
            var surface2 = new NurbsSurface(degreeU, degreeV, kvU2, kvV, cp2);
            //U0 V: (2,0,0),(2,1,0),(2,2,0)

            var joinedSurface = JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.U);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(joinedSurface.DegreeU, Is.EqualTo(degreeU));
                Assert.That(joinedSurface.DegreeV, Is.EqualTo(degreeV));
                Assert.That(joinedSurface.ControlPoints, Has.Length.GreaterThan(cp1.Length));
                Assert.That(joinedSurface.ControlPoints[0], Has.Length.EqualTo(3));
            }

            // check a few sample points
            double[] samples_extend_dir = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5,1.75,2];
            double[] samples_other_dir = [0, 0.5, 1];
            foreach (var u in samples_extend_dir)
            {
                foreach (var v in samples_other_dir)
                {
                    Vector3Double estimatedPos;
                    if (u <= 1.0)
                    {
                        estimatedPos = surface1.GetPos(u, v);
                    }
                    else
                    {
                        estimatedPos = surface2.GetPos(u - 1.0, v);
                    }
                    var joinedPos = joinedSurface.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(joinedPos.X, Is.EqualTo(estimatedPos.X).Within(1e-6));
                        Assert.That(joinedPos.Y, Is.EqualTo(estimatedPos.Y).Within(1e-6));
                        Assert.That(joinedPos.Z, Is.EqualTo(estimatedPos.Z).Within(1e-6));
                    }
                }
            }

        }

        [Test]
        public void JoinSurfaces_VDirection_BasicTest()
        {
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV1 = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV1 = new KnotVector(knotsV1, degreeV);

            var cp1 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp1[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp1[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }
            var surface1 = new NurbsSurface(degreeU, degreeV, kvU, kvV1, cp1);
            // V end U: (0,2,0),(1,2,0),(2,2,0)

            var knotsV2 = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvV2 = new KnotVector(knotsV2, degreeV);

            var cp2 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp2[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp2[i][j] = new ControlPoint(i, j + 2, 0, 1);
                }
            }
            var surface2 = new NurbsSurface(degreeU, degreeV, kvU, kvV2, cp2);
            // V0 U: (0,2,0),(1,2,0),(2,2,0)

            var joinedSurface = JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.V);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(joinedSurface.DegreeU, Is.EqualTo(degreeU));
                Assert.That(joinedSurface.DegreeV, Is.EqualTo(degreeV));
                Assert.That(joinedSurface.ControlPoints, Has.Length.EqualTo(3));
                Assert.That(joinedSurface.ControlPoints[0], Has.Length.GreaterThan(cp1[0].Length));
            }

            // check a few sample points
            double[] samples_extend_dir = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2];
            double[] samples_other_dir = [0, 0.5, 1];
            foreach (var u in  samples_other_dir)
            {
                foreach (var v in samples_extend_dir)
                {
                    Vector3Double estimatedPos;
                    if (u <= 1.0)
                    {
                        estimatedPos = surface1.GetPos(u, v);
                    }
                    else
                    {
                        estimatedPos = surface2.GetPos(u, v - 1.0);
                    }
                    var joinedPos = joinedSurface.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(joinedPos.X, Is.EqualTo(estimatedPos.X).Within(1e-6));
                        Assert.That(joinedPos.Y, Is.EqualTo(estimatedPos.Y).Within(1e-6));
                        Assert.That(joinedPos.Z, Is.EqualTo(estimatedPos.Z).Within(1e-6));
                    }
                }
            }

        }

        [Test]
        public void JoinSurfaces_SplitJoinRoundtrip_U()
        {
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var cp = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp[i][j] = new ControlPoint(i, j, Math.Sin(i) * Math.Cos(j), 1);
                }
            }
            var originalSurface = new NurbsSurface(degreeU, degreeV, kvU, kvV, cp);

            var (surface1, surface2) = SplitOperator.SplitSurface(originalSurface, 0.5, SurfaceDirection.U);
            var joinedSurface = JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.U);

            double[] samples = [0, 0.25, 0.5, 0.75, 1];
            foreach (var u in samples)
            {
                foreach (var v in samples)
                {
                    var originalPos = originalSurface.GetPos(u, v);
                    var joinedPos = joinedSurface.GetPos(u, v);

                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(joinedPos.X, Is.EqualTo(originalPos.X).Within(1e-6));
                        Assert.That(joinedPos.Y, Is.EqualTo(originalPos.Y).Within(1e-6));
                        Assert.That(joinedPos.Z, Is.EqualTo(originalPos.Z).Within(1e-6));
                    }
                }
            }
        }

        [Test]
        public void JoinSurfaces_SplitJoinRoundtrip_V()
        {
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var cp = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp[i][j] = new ControlPoint(i, j, Math.Sin(i) * Math.Cos(j), 1);
                }
            }
            var originalSurface = new NurbsSurface(degreeU, degreeV, kvU, kvV, cp);

            var (surface1, surface2) = SplitOperator.SplitSurface(originalSurface, 0.5, SurfaceDirection.V);
            var joinedSurface = JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.V);

            double[] samples = [0, 0.25, 0.5, 0.75, 1];
            foreach (var u in samples)
            {
                foreach (var v in samples)
                {
                    var originalPos = originalSurface.GetPos(u, v);
                    var joinedPos = joinedSurface.GetPos(u, v);

                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(joinedPos.X, Is.EqualTo(originalPos.X).Within(1e-6));
                        Assert.That(joinedPos.Y, Is.EqualTo(originalPos.Y).Within(1e-6));
                        Assert.That(joinedPos.Z, Is.EqualTo(originalPos.Z).Within(1e-6));
                    }
                }
            }
        }

        [Test]
        public void JoinSurfaces_DifferentDegrees_AutoElevation()
        {
            // Create two surfaces with same V degree but different U degrees
            // They must have the same number of control points in V direction
            int degreeU1 = 2;
            int degreeU2 = 3;
            int degreeV = 2;
            var knotsU1 = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsU2 = new double[] { 0, 0, 0, 0, 1, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU1 = new KnotVector(knotsU1, degreeU1);
            var kvU2 = new KnotVector(knotsU2, degreeU2);
            var kvV = new KnotVector(knotsV, degreeV);

            // Surface1: 3x3 control points
            var cp1 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp1[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp1[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }
            var surface1 = new NurbsSurface(degreeU1, degreeV, kvU1, kvV, cp1);
            // U end V: (2,0,0),(2,1,0),(2,2,0)

            // Surface2: 4x3 control points (different U degree, same V degree and CP count)
            var cp2 = new ControlPoint[4][];
            for (int i = 0; i < 4; i++)
            {
                cp2[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp2[i][j] = new ControlPoint(i + 2, j, 0, 1);
                }
            }
            var surface2 = new NurbsSurface(degreeU2, degreeV, kvU2, kvV, cp2);
            // U0 V: (2,0,0),(2,1,0),(2,2,0)

            // Join in U direction - should auto-elevate degreeU to 3
            var joinedSurface = JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.U);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(joinedSurface.DegreeU, Is.EqualTo(Math.Max(degreeU1, degreeU2)));
                Assert.That(joinedSurface.DegreeV, Is.EqualTo(degreeV));
            }

            // check a few sample points
            double[] samples_extend_dir = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2];
            double[] samples_other_dir = [0, 0.5, 1];
            foreach (var u in samples_extend_dir)
            {
                foreach (var v in samples_other_dir)
                {
                    Vector3Double estimatedPos;
                    if (u <= 1.0)
                    {
                        estimatedPos = surface1.GetPos(u, v);
                    }
                    else
                    {
                        estimatedPos = surface2.GetPos(u - 1.0, v);
                    }
                    var joinedPos = joinedSurface.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(joinedPos.X, Is.EqualTo(estimatedPos.X).Within(1e-6));
                        Assert.That(joinedPos.Y, Is.EqualTo(estimatedPos.Y).Within(1e-6));
                        Assert.That(joinedPos.Z, Is.EqualTo(estimatedPos.Z).Within(1e-6));
                    }
                }
            }
        }

        [Test]
        public void JoinSurfaces_EdgeMismatch_ThrowsException()
        {
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var cp1 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp1[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp1[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }
            var surface1 = new NurbsSurface(degreeU, degreeV, kvU, kvV, cp1);
            // U end: (2,0,0),(2,1,0),(2,2,0)

            var cp2 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp2[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp2[i][j] = new ControlPoint(i, j, 10, 1);
                }
            }
            var surface2 = new NurbsSurface(degreeU, degreeV, kvU, kvV, cp2);
            // U start: (0,0,10),(0,1,10),(0,2,10) - mismatched edge

            Assert.Throws<InvalidOperationException>(() =>
                JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.U));
        }

        [Test]
        public void JoinSurfaces_SizeMismatch_ThrowsException()
        {
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV1 = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV2 = new double[] { 0, 0, 0, 0, 1, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV1 = new KnotVector(knotsV1, degreeV);
            var kvV2 = new KnotVector(knotsV2, 3);

            var cp1 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp1[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    cp1[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }
            var surface1 = new NurbsSurface(degreeU, degreeV, kvU, kvV1, cp1);
            //U end: (2,0,0),(2,1,0),(2,2,0)

            var cp2 = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                cp2[i] = new ControlPoint[4];
                for (int j = 0; j < 4; j++)
                {
                    cp2[i][j] = new ControlPoint(i + 2, j, 0, 1);
                }
            }
            var surface2 = new NurbsSurface(degreeU, 3, kvU, kvV2, cp2);
            //U start: (2,0,0),(2,1,0),(2,2,0),(2,3,0) - mismatched size in V direction

            Assert.Throws<InvalidOperationException>(() =>
                JoinOperator.JoinSurfaces(surface1, surface2, SurfaceDirection.U));
        }

        [Test]
        public void JoinSurfaces_SplitTest()
        {
            int degreeU = 3;
            int degreeV = 3;
            double[] knotsU = [0, 0, 0, 0, 0.5, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 0.5, 1, 1, 1, 1];
            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);
            ControlPoint[][] controlPoints =
            [
                [
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1),
                new ControlPoint(2.0, 0.0, 0.0, 1),
                new ControlPoint(3.0, 0.0, 0.0, 1),
                new ControlPoint(4.0, 0.0, 0.0, 1)
                ],
                [
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, -1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1),
                new ControlPoint(3.0, 1.0, -3.0, 1),
                new ControlPoint(4.0, 1.0, 0.5, 1)
                ],
                [
                new ControlPoint(0.0, 2.0, 1.5, 1),
                new ControlPoint(1.0, 2.0, 2.5, 1),
                new ControlPoint(2.0, 2.0, 3.5, 0.7),
                new ControlPoint(3.0, 2.0, 3.0, 1),
                new ControlPoint(4.0, 2.0, 0.0, 1)
                ],
                [
                new ControlPoint(0.0, 3.0, 0.5, 1),
                new ControlPoint(1.5, 3.0, -1.5, 1),
                new ControlPoint(2.5, 3.0, 2.0 ,1),
                new ControlPoint(3.5, 3.0, -1.5, 1),
                new ControlPoint(4.5, 3.0, -1.0, 1)
                ],
                [
                new ControlPoint(0.0, 4.0, 0.5, 1),
                new ControlPoint(1.0, 4.0, 0.5, 1),
                new ControlPoint(2.0, 4.0, 0.0, 1),
                new ControlPoint(3.0, 4.0, 0.0, 1),
                new ControlPoint(4.0, 4.0, 0.0, 1)
                ],
            ];

            var surface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var (s1, s2) = SplitOperator.SplitSurface(surface, 0.5, SurfaceDirection.U);
            var joinedSurface = JoinOperator.JoinSurfaces(s1, s2, SurfaceDirection.U);

            // check a few sample points
            // Compare with original surface instead of split surfaces
            double[] samples = [0, 0.25, 0.5, 0.75, 1];
            foreach (var u in samples)
            {
                foreach (var v in samples)
                {
                    var originalPos = surface.GetPos(u, v);
                    var joinedPos = joinedSurface.GetPos(u, v);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(joinedPos.X, Is.EqualTo(originalPos.X).Within(1e-6));
                        Assert.That(joinedPos.Y, Is.EqualTo(originalPos.Y).Within(1e-6));
                        Assert.That(joinedPos.Z, Is.EqualTo(originalPos.Z).Within(1e-6));
                    }
                }
            }

        }

        static void TestOutIGES(List<NurbsCurve> geometries, string filePath="testcurve.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);

            var success = IGESExporter.ExportAsync(geometries, stream);
        }
    }
}
