using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using NurbsSharp.IO.IGES;

namespace UnitTests.Operation
{
    [TestFixture]
    public class SplitOperatorTest
    {
        [Test]
        public void SplitCurve_BasicTest()
        {
            // Arrange: Create a simple NURBS curve
            int degree = 2;
            var knots = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotVector = new KnotVector(knots, degree);
            var controlPoints = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(1, 1, 0, 1),
                new ControlPoint(2, 0, 0, 1)
            };
            var curve = new NurbsCurve(degree, knotVector, controlPoints);

            // Act: Split at u = 0.5
            var (leftCurve, rightCurve) = SplitOperator.SplitCurve(curve, 0.5);

            using (Assert.EnterMultipleScope())
            {
                // Assert: Verify dimensions
                Assert.That(leftCurve.Degree, Is.EqualTo(degree));
                Assert.That(rightCurve.Degree, Is.EqualTo(degree));
                Assert.That(leftCurve.ControlPoints, Is.Not.Empty);
                Assert.That(rightCurve.ControlPoints, Is.Not.Empty);
            }

            // Verify continuity at split point
            var leftEnd = leftCurve.GetPos(leftCurve.KnotVector.Knots[^1]);
            var rightStart = rightCurve.GetPos(rightCurve.KnotVector.Knots[0]);
            Assert.That(leftEnd.DistanceTo(rightStart), Is.LessThan(1e-6));

            double[] split_us = [0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var u in split_us)
            {
                (leftCurve, rightCurve) = SplitOperator.SplitCurve(curve, u);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    var pos_original = curve.GetPos(s);

                    Vector3Double pos;
                    if (s < u)//check s belong left or right
                        pos = leftCurve.GetPos(s);
                    else
                        pos = rightCurve.GetPos(s);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                        Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                        Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                    }

                }

            }
        }


        [Test]
        public void SplitOperator_SplitCurve_TestA()
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
            double[] split_us = [0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var u in split_us)
            {
                var (leftCurve, rightCurve) = SplitOperator.SplitCurve(curve, u);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    var pos_original = curve.GetPos(s);

                    Vector3Double pos;
                    if (s < u)//check s belong left or righ
                        pos = leftCurve.GetPos(s);
                    else
                        pos = rightCurve.GetPos(s);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                        Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                        Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                    }

                }

            }
        }


        [Test]
        public void SplitSurface_UDirection_BasicTest()
        {
            // Arrange: Create a simple NURBS surface
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[3][];
            controlPoints[0] =
            [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(0, 1, 0, 1),
                new ControlPoint(0, 2, 0, 1)
            ];
            controlPoints[1] =
            [
                new ControlPoint(1, 0, 1, 1),
                new ControlPoint(1, 1, 1, 1),
                new ControlPoint(1, 2, 1, 1)
            ];
            controlPoints[2] =
            [
                new ControlPoint(2, 0, 0, 1),
                new ControlPoint(2, 1, 0, 1),
                new ControlPoint(2, 2, 0, 1)
            ];

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);

            // Act: Split in U direction at u = 0.5
            var (surface1, surface2) = SplitOperator.SplitSurface(surface, 0.5, SplitDirection.U);

            using (Assert.EnterMultipleScope())
            {
                // Assert: Verify degrees are preserved
                Assert.That(surface1.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface1.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface2.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface2.DegreeV, Is.EqualTo(degreeV));

                // Verify V dimensions are preserved
                Assert.That(surface1.ControlPoints[0], Has.Length.EqualTo(3));
                Assert.That(surface2.ControlPoints[0], Has.Length.EqualTo(3));

                // Verify U dimensions are split
                Assert.That(surface1.ControlPoints, Is.Not.Empty);
                Assert.That(surface2.ControlPoints, Is.Not.Empty);
            }

            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var u in split_us)
            {
                (surface1, surface2) = SplitOperator.SplitSurface(surface,u, SplitDirection.U);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    foreach (var s2 in sample_points)
                    {
                        var pos_original = surface.GetPos(s, s2);

                        Vector3Double pos;
                        if (s < u)//check s belong left or right
                            pos = surface1.GetPos(s, s2);
                        else
                            pos = surface2.GetPos(s, s2);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                            Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                            Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                        }
                    }

                }

            }

        }

        [Test]
        public void SplitSurface_VDirection_BasicTest()
        {
            // Arrange: Create a simple NURBS surface
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[3][];
            controlPoints[0] =
            [
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(0, 1, 0, 1),
                new ControlPoint(0, 2, 0, 1)
            ];
            controlPoints[1] =
            [
                new ControlPoint(1, 0, 1, 1),
                new ControlPoint(1, 1, 1, 1),
                new ControlPoint(1, 2, 1, 1)
            ];
            controlPoints[2] =
            [
                new ControlPoint(2, 0, 0, 1),
                new ControlPoint(2, 1, 0, 1),
                new ControlPoint(2, 2, 0, 1)
            ];

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);

            // Act: Split in V direction at v = 0.5
            var (surface1, surface2) = SplitOperator.SplitSurface(surface, 0.5, SplitDirection.V);

            using (Assert.EnterMultipleScope())
            {
                // Assert: Verify degrees are preserved
                Assert.That(surface1.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface1.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface2.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface2.DegreeV, Is.EqualTo(degreeV));

                // Verify U dimensions are preserved
                Assert.That(surface1.ControlPoints, Has.Length.EqualTo(3));
                Assert.That(surface2.ControlPoints, Has.Length.EqualTo(3));

                // Verify V dimensions are split
                Assert.That(surface1.ControlPoints[0], Is.Not.Empty);
                Assert.That(surface2.ControlPoints[0], Is.Not.Empty);
            }

            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var v in split_us)
            {
                (surface1, surface2) = SplitOperator.SplitSurface(surface, v, SplitDirection.V);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    foreach (var s2 in sample_points)
                    {
                        var pos_original = surface.GetPos(s, s2);

                        Vector3Double pos;
                        if (s2 < v)//check s belong left or righ
                            pos = surface1.GetPos(s, s2);
                        else
                            pos = surface2.GetPos(s, s2);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                            Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                            Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                        }
                    }

                }

            }
        }

        [Test]
        public void SplitSurface_UDirection_GeometryContinuity()
        {
            // Arrange: Create a test surface
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                controlPoints[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    controlPoints[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);
            double splitU = 0.5;

            // Act
            var (surface1, surface2) = SplitOperator.SplitSurface(surface, splitU, SplitDirection.U);

            // Assert: Verify geometry continuity at split
            for (double v = 0; v <= 1; v += 0.1)
            {
                var point1 = surface1.GetPos(surface1.KnotVectorU.Knots[^1], v);
                var point2 = surface2.GetPos(surface2.KnotVectorU.Knots[0], v);
                Assert.That(point1.DistanceTo(point2), Is.LessThan(1e-6),
                    $"Surfaces are not continuous at u={splitU}, v={v}");
            }

            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var u in split_us)
            {
                (surface1, surface2) = SplitOperator.SplitSurface(surface, u, SplitDirection.U);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    foreach (var s2 in sample_points)
                    {
                        var pos_original = surface.GetPos(s, s2);

                        Vector3Double pos;
                        if (s < u)//check s belong left or righ
                            pos = surface1.GetPos(s, s2);
                        else
                            pos = surface2.GetPos(s, s2);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                            Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                            Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                        }
                    }

                }

            }
        }

        [Test]
        public void SplitSurface_VDirection_GeometryContinuity()
        {
            // Arrange: Create a test surface
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                controlPoints[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    controlPoints[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);
            double splitV = 0.5;

            // Act
            var (surface1, surface2) = SplitOperator.SplitSurface(surface, splitV, SplitDirection.V);

            // Assert: Verify geometry continuity at split
            for (double u = 0; u <= 1; u += 0.1)
            {
                var point1 = surface1.GetPos(u, surface1.KnotVectorV.Knots[^1]);
                var point2 = surface2.GetPos(u, surface2.KnotVectorV.Knots[0]);
                Assert.That(point1.DistanceTo(point2), Is.LessThan(1e-6),
                    $"Surfaces are not continuous at u={u}, v={splitV}");
            }

            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var v in split_us)
            {
                (surface1, surface2) = SplitOperator.SplitSurface(surface, v, SplitDirection.V);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    foreach (var s2 in sample_points)
                    {
                        var pos_original = surface.GetPos(s, s2);

                        Vector3Double pos;
                        if (s2 < v)//check s belong left or righ
                            pos = surface1.GetPos(s, s2);
                        else
                            pos = surface2.GetPos(s, s2);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                            Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                            Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                        }
                    }

                }

            }
        }

        [Test]
        public void SplitSurface_UDirection_DifferentDegrees()
        {
            // Arrange: Create a surface with different U and V degrees
            int degreeU = 3;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 0, 1, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[4][];
            for (int i = 0; i < 4; i++)
            {
                controlPoints[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    controlPoints[i][j] = new ControlPoint(i, j, Math.Sin(i) * Math.Cos(j) * 5, 1);
                }
            }

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);

            // Act
            var (surface1, surface2) = SplitOperator.SplitSurface(surface, 0.5, SplitDirection.U);

            using (Assert.EnterMultipleScope())
            {
                // Assert
                Assert.That(surface1.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface1.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface2.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface2.DegreeV, Is.EqualTo(degreeV));
            }

            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var u in split_us)
            {
                (surface1, surface2) = SplitOperator.SplitSurface(surface, u, SplitDirection.U);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    foreach (var s2 in sample_points)
                    {
                        var pos_original = surface.GetPos(s, s2);

                        Vector3Double pos;
                        if (s < u)//check s belong left or righ
                            pos = surface1.GetPos(s, s2);
                        else
                            pos = surface2.GetPos(s, s2);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                            Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                            Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                        }
                    }

                }

            }
        }

        [Test]
        public void SplitSurface_OutOfRange_ThrowsException()
        {
            // Arrange
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[3][];
            for (int i = 0; i < 3; i++)
            {
                controlPoints[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    controlPoints[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);

            // Act & Assert
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                SplitOperator.SplitSurface(surface, 1.5, SplitDirection.U));
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                SplitOperator.SplitSurface(surface, -0.5, SplitDirection.V));
        }

        [Test]
        public void SplitSurface_NullSurface_ThrowsException()
        {
            // Act & Assert
            Assert.Throws<ArgumentNullException>(() =>
                SplitOperator.SplitSurface(null!, 0.5, SplitDirection.U));
        }

        [Test]
        public void SplitSurface_AtExistingKnot()
        {
            // Arrange: Create a surface with knots
            int degreeU = 2;
            int degreeV = 2;
            var knotsU = new double[] { 0, 0, 0, 0.5, 1, 1, 1 };
            var knotsV = new double[] { 0, 0, 0, 1, 1, 1 };
            var kvU = new KnotVector(knotsU, degreeU);
            var kvV = new KnotVector(knotsV, degreeV);

            var controlPoints = new ControlPoint[4][];
            for (int i = 0; i < 4; i++)
            {
                controlPoints[i] = new ControlPoint[3];
                for (int j = 0; j < 3; j++)
                {
                    controlPoints[i][j] = new ControlPoint(i, j, 0, 1);
                }
            }

            var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);

            // Act: Split at existing knot
            var (surface1, surface2) = SplitOperator.SplitSurface(surface, 0.5, SplitDirection.U);

            using (Assert.EnterMultipleScope())
            {
                // Assert: Should succeed
                Assert.That(surface1.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface2.DegreeU, Is.EqualTo(degreeU));
            }

            //TestOutIGES([surface, surface1, surface2]);

            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];

            foreach (var u in split_us)
            {
                (surface1, surface2) = SplitOperator.SplitSurface(surface, u, SplitDirection.U);

                // Verify continuity at split point
                foreach (var s in sample_points)
                {
                    foreach (var s2 in sample_points)
                    {
                        var pos_original = surface.GetPos(s, s2);

                        Vector3Double pos;
                        if (s < u)//check s belong left or righ
                            pos = surface1.GetPos(s, s2);
                        else
                            pos = surface2.GetPos(s, s2);
                        using (Assert.EnterMultipleScope())
                        {
                            Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                            Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                            Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                        }
                    }

                }

            }
        }

        [Test]
        public void SplitSurfTestA()
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
            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            double[] split_us = [0.1, 0.3, 0.5, 0.75];
            double[] split_vs = [0.1, 0.25, 0.5, 0.75];
            double[] sample_points = [0, 0.1, 0.2, 0.3, 0.5, 0.55, 0.7, 0.9, 0.9999, 1];
            foreach (var u in split_us)
            {
                foreach(var v in split_vs)
                {
                    var (surfaceU1, surfaceU2) = SplitOperator.SplitSurface(nurbsSurface, u, SplitDirection.U);
                    var (surfaceU1V1, surfaceU1V2) = SplitOperator.SplitSurface(surfaceU1, v, SplitDirection.V);
                    var (surfaceU2V1, surfaceU2V2) = SplitOperator.SplitSurface(surfaceU2, v, SplitDirection.V);
                    // Verify continuity at split point
                    foreach (var s in sample_points)
                    {
                        foreach (var s2 in sample_points)
                        {
                            var pos_original = nurbsSurface.GetPos(s, s2);
                            Vector3Double pos;
                            if (s < u)//check s belong left or righ
                            {
                                if (s2 < v)
                                    pos = surfaceU1V1.GetPos(s, s2);
                                else
                                    pos = surfaceU1V2.GetPos(s, s2);
                            }
                            else
                            {
                                if (s2 < v)
                                    pos = surfaceU2V1.GetPos(s, s2);
                                else
                                    pos = surfaceU2V2.GetPos(s, s2);
                            }
                            using (Assert.EnterMultipleScope())
                            {
                                Assert.That(pos.X, Is.EqualTo(pos_original.X).Within(0.00000001));
                                Assert.That(pos.Y, Is.EqualTo(pos_original.Y).Within(0.00000001));
                                Assert.That(pos.Z, Is.EqualTo(pos_original.Z).Within(0.00000001));
                            }
                        }
                    }
                }
            }

        }

        static void TestOutIGES(List<NurbsSurface> geometries, string filePath = "testsplit.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);

            var success = IGESExporter.ExportAsync(geometries, stream);
        }
    }
}
