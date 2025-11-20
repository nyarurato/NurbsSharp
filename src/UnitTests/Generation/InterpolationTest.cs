using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.IO;
using NurbsSharp.Tesselation;
using NurbsSharp.IO.IGES;
using NurbsSharp.Generation.Interpolation;
using System;
using System.Collections.Generic;
using System.IO;

namespace UnitTests.Generation
{
    [TestFixture]
    public class InterpolationTest
    {
        [Test]
        public void GlobalInterpolation_SimpleQuadratic()
        {
            // Arrange: 3 points for a quadratic curve
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0),
                new Vector3Double(2, 0, 0)
            };
            int degree = 2;

            // Act
            var curve = GlobalInterpolator.InterpolateCurve(points, degree);

            // Assert
            Assert.That(curve, Is.Not.Null);
            Assert.That(curve.Degree, Is.EqualTo(degree));
            Assert.That(curve.ControlPoints.Length, Is.EqualTo(3));

            // Verify curve passes through the points
            double[] parameters = InterpolationHelper.ComputeParameters(points, ParameterizationType.Chord);
            for (int i = 0; i < points.Length; i++)
            {
                var evaluated = curve.GetPos(parameters[i]);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(evaluated.X, Is.EqualTo(points[i].X).Within(1e-10));
                    Assert.That(evaluated.Y, Is.EqualTo(points[i].Y).Within(1e-10));
                    Assert.That(evaluated.Z, Is.EqualTo(points[i].Z).Within(1e-10));
                }
            }
        }

        [Test]
        public void GlobalInterpolation_CubicCurve()
        {
            // Arrange: 5 points for a cubic curve
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 2, 0),
                new Vector3Double(2, 3, 0),
                new Vector3Double(3, 2, 0),
                new Vector3Double(4, 0, 0)
            };
            int degree = 3;

            // Act
            var curve = GlobalInterpolator.InterpolateCurve(points, degree);

            // Assert
            Assert.That(curve, Is.Not.Null);
            Assert.That(curve.Degree, Is.EqualTo(degree));
            Assert.That(curve.ControlPoints.Length, Is.EqualTo(5));

            // Verify curve passes through the points
            double[] parameters = InterpolationHelper.ComputeParameters(points, ParameterizationType.Chord);
            for (int i = 0; i < points.Length; i++)
            {
                var evaluated = curve.GetPos(parameters[i]);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(evaluated.X, Is.EqualTo(points[i].X).Within(1e-8));
                    Assert.That(evaluated.Y, Is.EqualTo(points[i].Y).Within(1e-8));
                    Assert.That(evaluated.Z, Is.EqualTo(points[i].Z).Within(1e-8));
                }
            }
        }

        [Test]
        public void GlobalInterpolation_DifferentParameterizations()
        {
            // Arrange
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0),
                new Vector3Double(3, 1, 0),
                new Vector3Double(4, 0, 0)
            };
            int degree = 3;

            // Act: Test all three parameterization types
            var optionsChord = new InterpolationOptions { ParameterizationType = ParameterizationType.Chord };
            var optionsCentripetal = new InterpolationOptions { ParameterizationType = ParameterizationType.Centripetal };
            var optionsUniform = new InterpolationOptions { ParameterizationType = ParameterizationType.Uniform };

            var curveChord = GlobalInterpolator.InterpolateCurve(points, degree, optionsChord);
            var curveCentripetal = GlobalInterpolator.InterpolateCurve(points, degree, optionsCentripetal);
            var curveUniform = GlobalInterpolator.InterpolateCurve(points, degree, optionsUniform);

            // Assert: All should produce valid curves
            using (Assert.EnterMultipleScope())
            {
                Assert.That(curveChord, Is.Not.Null);
                Assert.That(curveCentripetal, Is.Not.Null);
                Assert.That(curveUniform, Is.Not.Null);
            }

            // All curves should pass through endpoints
            var startPoint = points[0];
            var endPoint = points[points.Length - 1];

            var evalChordStart = curveChord.GetPos(curveChord.KnotVector.Knots[degree]);
            var evalChordEnd = curveChord.GetPos(curveChord.KnotVector.Knots[curveChord.KnotVector.Length - degree - 1]);

            Assert.That(evalChordStart.X, Is.EqualTo(startPoint.X).Within(1e-8));
            Assert.That(evalChordEnd.X, Is.EqualTo(endPoint.X).Within(1e-8));
        }

        [Test]
        public void GlobalInterpolation_ThrowsOnInvalidInput()
        {
            // Test with too few points
            var tooFewPoints = new Vector3Double[] { new Vector3Double(0, 0, 0) };
            Assert.Throws<ArgumentException>(() => GlobalInterpolator.InterpolateCurve(tooFewPoints, 2));

            // Test with degree too high
            var points = new Vector3Double[]
            {
                new Vector3Double(0, 0, 0),
                new Vector3Double(1, 1, 0)
            };
            Assert.Throws<ArgumentException>(() => GlobalInterpolator.InterpolateCurve(points, 3));

            // Test with null points
            Assert.Throws<ArgumentNullException>(() => GlobalInterpolator.InterpolateCurve(null!, 2));
        }

        // ===== Surface Interpolation Tests =====

        [Test]
        public void GlobalSurfaceInterpolation_SimpleBilinear()
        {
            // Arrange: 2x2 point grid for a bilinear surface (degree 1x1)
            var points = new Vector3Double[][]
            {
                new Vector3Double[]
                {
                    new Vector3Double(0, 0, 0),
                    new Vector3Double(0, 1, 0)
                },
                new Vector3Double[]
                {
                    new Vector3Double(1, 0, 1),
                    new Vector3Double(1, 1, 1)
                }
            };
            int degreeU = 1;
            int degreeV = 1;

            // Act
            var surface = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV);

            // Assert
            Assert.That(surface, Is.Not.Null);
            Assert.That(surface.DegreeU, Is.EqualTo(degreeU));
            Assert.That(surface.DegreeV, Is.EqualTo(degreeV));
            Assert.That(surface.ControlPoints.Length, Is.EqualTo(2));
            Assert.That(surface.ControlPoints[0].Length, Is.EqualTo(2));

            // Verify surface passes through the corner points
            var (uk, vl) = InterpolationHelper.ComputeParametersSurface(points, ParameterizationType.Chord);

            for (int u = 0; u < points.Length; u++)
            {
                for (int v = 0; v < points[u].Length; v++)
                {
                    var evaluated = surface.GetPos(uk[u], vl[v]);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(evaluated.X, Is.EqualTo(points[u][v].X).Within(1e-10));
                        Assert.That(evaluated.Y, Is.EqualTo(points[u][v].Y).Within(1e-10));
                        Assert.That(evaluated.Z, Is.EqualTo(points[u][v].Z).Within(1e-10));
                    }
                }
            }

        }

        [Test]
        public void GlobalSurfaceInterpolation_Bicubic()
        {
            // Arrange: 5x5 point grid for a bicubic surface (degree 3x3)
            var points = new Vector3Double[5][];
            for (int i = 0; i < 5; i++)
            {
                points[i] = new Vector3Double[5];
                for (int j = 0; j < 5; j++)
                {
                    double x = i;
                    double y = j;
                    double z = Math.Sin(x * 0.5) * Math.Cos(y * 0.5);
                    points[i][j] = new Vector3Double(x, y, z);
                }
            }
            int degreeU = 3;
            int degreeV = 3;

            // Act
            var surface = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV);

            // Assert
            Assert.That(surface, Is.Not.Null);
            Assert.That(surface.DegreeU, Is.EqualTo(degreeU));
            Assert.That(surface.DegreeV, Is.EqualTo(degreeV));
            Assert.That(surface.ControlPoints.Length, Is.EqualTo(5));
            Assert.That(surface.ControlPoints[0].Length, Is.EqualTo(5));

            // Verify surface passes through all points
            var (uk, vl) = InterpolationHelper.ComputeParametersSurface(points, ParameterizationType.Chord);

            for (int u = 0; u < points.Length; u++)
            {
                for (int v = 0; v < points[u].Length; v++)
                {
                    var evaluated = surface.GetPos(uk[u], vl[v]);
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(evaluated.X, Is.EqualTo(points[u][v].X).Within(1e-8));
                        Assert.That(evaluated.Y, Is.EqualTo(points[u][v].Y).Within(1e-8));
                        Assert.That(evaluated.Z, Is.EqualTo(points[u][v].Z).Within(1e-8));
                    }
                }
            }
        }

        [Test]
        public void GlobalSurfaceInterpolation_MixedDegree()
        {
            // Arrange: 4x5 point grid with degree 2 in U, degree 3 in V
            var points = new Vector3Double[4][];
            for (int i = 0; i < 4; i++)
            {
                points[i] = new Vector3Double[5];
                for (int j = 0; j < 5; j++)
                {
                    points[i][j] = new Vector3Double(i, j, i + j * 0.5);
                }
            }
            int degreeU = 2;
            int degreeV = 3;

            // Act
            var surface = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV);

            // Assert
            using (Assert.EnterMultipleScope())
            {
                Assert.That(surface, Is.Not.Null);
                Assert.That(surface.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface.ControlPoints.Length, Is.EqualTo(4));
                Assert.That(surface.ControlPoints[0].Length, Is.EqualTo(5));
            }

            // Verify interpolation at corners
            var (uk, vl) = InterpolationHelper.ComputeParametersSurface(points, ParameterizationType.Chord);

            var corners = new (int u, int v)[] { (0, 0), (3, 0), (0, 4), (3, 4) };
            foreach (var (u, v) in corners)
            {
                var evaluated = surface.GetPos(uk[u], vl[v]);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(evaluated.X, Is.EqualTo(points[u][v].X).Within(1e-8));
                    Assert.That(evaluated.Y, Is.EqualTo(points[u][v].Y).Within(1e-8));
                    Assert.That(evaluated.Z, Is.EqualTo(points[u][v].Z).Within(1e-8));
                }
            }
        }

        [Test]
        public void GlobalSurfaceInterpolation_DifferentParameterizations()
        {
            // Arrange
            var points = new Vector3Double[3][];
            for (int i = 0; i < 3; i++)
            {
                points[i] = new Vector3Double[3];
                for (int j = 0; j < 3; j++)
                {
                    points[i][j] = new Vector3Double(i * 2.0, j * 2.0, (i + j) * 0.5);
                }
            }
            int degreeU = 2;
            int degreeV = 2;

            // Act: Test all three parameterization types
            var optionsChord = new InterpolationOptions { ParameterizationType = ParameterizationType.Chord };
            var optionsCentripetal = new InterpolationOptions { ParameterizationType = ParameterizationType.Centripetal };
            var optionsUniform = new InterpolationOptions { ParameterizationType = ParameterizationType.Uniform };

            var surfaceChord = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV, optionsChord);
            var surfaceCentripetal = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV, optionsCentripetal);
            var surfaceUniform = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV, optionsUniform);

            // Assert: All should produce valid surfaces
            using (Assert.EnterMultipleScope())
            {
                Assert.That(surfaceChord, Is.Not.Null);
                Assert.That(surfaceCentripetal, Is.Not.Null);
                Assert.That(surfaceUniform, Is.Not.Null);

                // Verify structure
                Assert.That(surfaceChord.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surfaceChord.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surfaceChord.ControlPoints.Length, Is.EqualTo(3));
                Assert.That(surfaceChord.ControlPoints[0].Length, Is.EqualTo(3));
            }
        }

        [Test]
        public void GlobalSurfaceInterpolation_ThrowsOnInvalidInput()
        {
            // Test with null points
            Assert.Throws<ArgumentNullException>(() =>
                GlobalInterpolator.InterpolateSurf((Vector3Double[][])null!, 2, 2));

            // Test with too few rows
            var tooFewRows = new Vector3Double[][]
            {
                new Vector3Double[] { new Vector3Double(0, 0, 0), new Vector3Double(0, 1, 0) }
            };
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(tooFewRows, 1, 1));

            // Test with too few columns
            var tooFewCols = new Vector3Double[][]
            {
                new Vector3Double[] { new Vector3Double(0, 0, 0) },
                new Vector3Double[] { new Vector3Double(1, 0, 0) }
            };
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(tooFewCols, 1, 1));

            // Test with invalid grid (not rectangular)
            var invalidGrid = new Vector3Double[][]
            {
                new Vector3Double[] { new Vector3Double(0, 0, 0), new Vector3Double(0, 1, 0) },
                new Vector3Double[] { new Vector3Double(1, 0, 0) } // Different length
            };
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(invalidGrid, 1, 1));

            // Test with degree too high in U
            var validGrid = new Vector3Double[][]
            {
                new Vector3Double[] { new Vector3Double(0, 0, 0), new Vector3Double(0, 1, 0) },
                new Vector3Double[] { new Vector3Double(1, 0, 0), new Vector3Double(1, 1, 0) }
            };
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(validGrid, 3, 1));

            // Test with degree too high in V
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(validGrid, 1, 3));

            // Test with invalid degrees (< 1)
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(validGrid, 0, 1));
            Assert.Throws<ArgumentException>(() =>
                GlobalInterpolator.InterpolateSurf(validGrid, 1, 0));
        }

        [Test]
        public void GlobalSurfaceInterpolation_Cylinder()
        {
            double radius = 2.5;

            var points = new Vector3Double[4][];
            for (int i = 0; i < 4; i++)
            {
                points[i] = new Vector3Double[10];
                for (int j = 0; j < 10; j++)
                {
                    var rad = Math.PI * 2.0 * j / 9.0;
                    points[i][j] = new Vector3Double(radius*Math.Cos(rad), radius * Math.Sin(rad), i);
                }
            }
            int degreeU = 2;
            int degreeV = 3;

            var surface = GlobalInterpolator.InterpolateSurf(points, degreeU, degreeV);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(surface, Is.Not.Null);
                Assert.That(surface.DegreeU, Is.EqualTo(degreeU));
                Assert.That(surface.DegreeV, Is.EqualTo(degreeV));
                Assert.That(surface.ControlPoints.Length, Is.EqualTo(4));
                Assert.That(surface.ControlPoints[0].Length, Is.EqualTo(10));
            }

            for (int i = 0; i < 100; i++)
            {
                for(int j = 0; j < 100; j++)
                {
                    var u = surface.KnotVectorU.Knots[degreeU] + (surface.KnotVectorU.Knots[surface.KnotVectorU.Length - degreeU - 1] - surface.KnotVectorU.Knots[degreeU]) * i / 99.0;
                    var v = surface.KnotVectorV.Knots[degreeV] + (surface.KnotVectorV.Knots[surface.KnotVectorV.Length - degreeV - 1] - surface.KnotVectorV.Knots[degreeV]) * j / 99.0;
                    var pos = surface.GetPos(u, v);
                    var calc_radius = Math.Sqrt(pos.X * pos.X + pos.Y * pos.Y);
                    Assert.That(calc_radius, Is.EqualTo(radius).Within(0.02)); // interpolate error  
                }
            }
        }

        private void TestOutputIGES(List<NurbsSurface> surface, string filePath = "TestFace.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            var success = IGESExporter.ExportAsync(surface, stream);

        }
        private void TestOutputSTL(Mesh mesh, string filePath = "GeometryTestMesh.stl")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            var success = STLExporter.ExportAsync(mesh, stream);
        }
    }
}
