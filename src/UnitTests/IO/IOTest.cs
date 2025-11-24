using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.IO;
using NurbsSharp.IO.BMP;
using NurbsSharp.IO.IGES;
using NurbsSharp.Tesselation;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Generation;

namespace UnitTests.IO
{
    [TestFixture]
    public class IOTest
    {
        [SetUp]
        public void Setup()
        {
        }

        [Test]
        public async Task OutputObj()
        {
            // Create a simple NURBS surface
            int degreeU = 1;
            int degreeV = 1;
            var knotVectorU = new KnotVector([0, 0, 1, 1],degreeU);
            var knotVectorV = new KnotVector([0, 0, 1, 1],degreeV);
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                ],
                [
                    new ControlPoint(1, 0, 1),
                    new ControlPoint(1, 1, 1)
                ]
            ];

            string estimate_str = "v 0 0 0\n" +
                                 "v 0 1 0\n" +
                                 "v 1 0 1\n" +
                                 "v 1 1 1\n" +
                                 "f 1 3 4\n" +
                                 "f 1 4 2";

            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 2, 2);

            using var ms = new MemoryStream();
            await OBJExporter.ExportAsync(mesh, ms);
            ms.Seek(0, SeekOrigin.Begin);
            using var reader = new StreamReader(ms);
            string objData = reader.ReadToEnd();

            Assert.That(
                objData.Replace("\r\n", "\n").Trim(['\r', '\n', ' ']),
                Is.EqualTo(estimate_str.Trim(['\r', '\n', ' ']))
            );
        }

        [Test]
        public async Task OutputObjFile()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;

            double[] knotsU = [0, 0, 0, 0, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 1, 1, 1, 1];

            var knotVectorU = new KnotVector(knotsU,degreeU);
            var knotVectorV = new KnotVector(knotsV,degreeV);

            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 1.0, 1),
                    new ControlPoint(2.0, 0.0, 3.0, 1),
                    new ControlPoint(3.0, 0.0, 3.0, 1)

                ],
                [
                    new ControlPoint(0.0, 1.0, 0.5, 1),
                    new ControlPoint(1.0, 1.0, 1.5, 1),
                    new ControlPoint(2.0, 1.0, 4.0, 1),
                    new ControlPoint(3.0, 1.0, 3.0, 1)

                ],
                [
                    new ControlPoint(3.0, 1.0, 0.5, 1),
                    new ControlPoint(3.0, 1.0, 1.5, 1),
                    new ControlPoint(3.0, 1.0, 5.0, 1),
                    new ControlPoint(3.0, 1.0, 6.0, 1)

                ],
                [
                    new ControlPoint(3.0, 2.0, 0.5, 1),
                    new ControlPoint(3.0, 2.0, 1.5, 1),
                    new ControlPoint(3.0, 2.0, 5.0, 1),
                    new ControlPoint(3.0, 2.0, 7.0, 1)

                ],
            ];
            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 20, 20);
            using (var fs = new FileStream("test_output.obj", FileMode.Create, FileAccess.Write))
            {
                await OBJExporter.ExportAsync(mesh, fs);
            }
            Console.WriteLine($"OBJ file 'test_output.obj' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_output.obj"), Is.True);
        }

        [Test]
        public async Task OutputNurbsObjFile()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;

            double[] knotsU = [0, 0, 0, 0, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 1, 1, 1, 1];

            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);

            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 1.0, 1),
                    new ControlPoint(2.0, 0.0, 3.0, 1),
                    new ControlPoint(3.0, 0.0, 3.0, 1)

                ],
                [
                    new ControlPoint(0.0, 1.0, 0.5, 1),
                    new ControlPoint(1.0, 1.0, 1.5, 1),
                    new ControlPoint(2.0, 1.0, 4.0, 1),
                    new ControlPoint(3.0, 1.0, 3.0, 1)

                ],
                [
                    new ControlPoint(3.0, 1.0, 0.5, 1),
                    new ControlPoint(3.0, 1.0, 1.5, 1),
                    new ControlPoint(3.0, 1.0, 5.0, 1),
                    new ControlPoint(3.0, 1.0, 6.0, 1)

                ],
                [
                    new ControlPoint(3.0, 2.0, 0.5, 1),
                    new ControlPoint(3.0, 2.0, 1.5, 1),
                    new ControlPoint(3.0, 2.0, 5.0, 1),
                    new ControlPoint(3.0, 2.0, 7.0, 1)

                ],
            ];
            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);

            using (var fs = new FileStream("test_output_nurbs.obj", FileMode.Create, FileAccess.Write))
            {
                await OBJExporter.ExportAsync(nurbsSurface, fs);
            }
            Console.WriteLine($"OBJ file 'test_output_nurbs.obj' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_output_nurbs.obj"), Is.True);
        }

        [Test]
        public async Task STLExportTestA()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;

            double[] knotsU = [0, 0, 0, 0, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 1, 1, 1, 1];

            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);

            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 1.0, 1),
                    new ControlPoint(2.0, 0.0, 3.0, 1),
                    new ControlPoint(3.0, 0.0, 3.0, 1)

                ],
                [
                    new ControlPoint(0.0, 1.0, 0.5, 1),
                    new ControlPoint(1.0, 1.0, 1.5, 1),
                    new ControlPoint(2.0, 1.0, 4.0, 1),
                    new ControlPoint(3.0, 1.0, 3.0, 1)

                ],
                [
                    new ControlPoint(3.0, 1.0, 0.5, 1),
                    new ControlPoint(3.0, 1.0, 1.5, 1),
                    new ControlPoint(3.0, 1.0, 5.0, 1),
                    new ControlPoint(3.0, 1.0, 6.0, 1)

                ],
                [
                    new ControlPoint(3.0, 2.0, 0.5, 1),
                    new ControlPoint(3.0, 2.0, 1.5, 1),
                    new ControlPoint(3.0, 2.0, 5.0, 1),
                    new ControlPoint(3.0, 2.0, 7.0, 1)

                ],
            ];
            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 20, 20);
            using (var fs = new FileStream("test_output.stl", FileMode.Create, FileAccess.Write))
            {
                await STLExporter.ExportAsync(mesh, fs);
            }
            Console.WriteLine($"STL file 'test_output.stl' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_output.stl"), Is.True);
        }

        [Test]
        public async Task STLExportTestB()
        {
            // Rectangular NURBS surface
            int degreeU = 1;
            int degreeV = 1;
            double[] knotsU = [0, 0, 1, 1];
            double[] knotsV = [0, 0, 1, 1];
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 0.0, 1)
                ],
                [
                    new ControlPoint(0.0, 0.0, 1.5, 1),
                    new ControlPoint(1.0, 0.0, 1.5, 1)
                ],
            ];
            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);
            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 100, 100);
            using (var fs = new FileStream("test_outputB.stl", FileMode.Create, FileAccess.Write))
            {
                await STLExporter.ExportAsync(mesh, fs);
            }
            Console.WriteLine($"STL file 'test_outputB.stl' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_outputB.stl"), Is.True);
        }

        [Test]
        public async Task IgesExportTestA()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;

            double[] knotsU = [0, 0, 0, 0, 1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 1, 1, 1, 1];

            var knotVectorU = new KnotVector(knotsU, degreeU);
            var knotVectorV = new KnotVector(knotsV, degreeV);

            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0.0, 0.0, 0.0, 1),
                    new ControlPoint(1.0, 0.0, 1.0, 1),
                    new ControlPoint(2.0, 0.0, 3.0, 1),
                    new ControlPoint(3.0, 0.0, 3.0, 1)

                ],
                [
                    new ControlPoint(0.0, 1.0, 0.5, 1),
                    new ControlPoint(1.0, 1.0, 1.5, 1),
                    new ControlPoint(2.0, 1.0, 4.0, 1),
                    new ControlPoint(3.0, 1.0, 3.0, 1)

                ],
                [
                    new ControlPoint(3.0, 1.0, 0.5, 1),
                    new ControlPoint(3.0, 1.0, 1.5, 1),
                    new ControlPoint(3.0, 1.0, 5.0, 1),
                    new ControlPoint(3.0, 1.0, 6.0, 1)

                ],
                [
                    new ControlPoint(3.0, 2.0, 0.5, 1),
                    new ControlPoint(3.0, 2.0, 1.5, 1),
                    new ControlPoint(3.0, 2.0, 5.0, 1),
                    new ControlPoint(3.0, 2.0, 7.0, 1)

                ],
            ];
            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);

            using (var fs = new FileStream("test_output.igs", FileMode.Create, FileAccess.Write))
            {
                await IGESExporter.ExportAsync([nurbsSurface], fs);
            }
            Console.WriteLine($"IGES file 'test_output.igs' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_output.igs"), Is.True);
        }
    


    [Test]
        public async Task OutputIgesFileB()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;
            double[] knotsU = [0, 0, 0, 0, 0.5,1, 1, 1, 1];
            double[] knotsV = [0, 0, 0, 0, 0.5,1, 1, 1, 1];
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
            using (var fs = new FileStream("test_outputB.igs", FileMode.Create, FileAccess.Write))
            {
                await IGESExporter.ExportAsync([nurbsSurface], fs);
            }
            Console.WriteLine($"IGES file 'test_outputB.igs' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_outputB.igs"), Is.True);

        }

        [Test]
        public async Task IgesExportCurveTest()
        {
            // Create a simple NURBS curve
            int degree = 3;
            double[] knots = [0, 0, 0, 0, 1, 1, 1, 1];
            var knotVector = new KnotVector(knots, degree);

            ControlPoint[] controlPoints =
            [
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 2.0, 1.0, 1),
                new ControlPoint(3.0, 3.0, 0.0, 1),
                new ControlPoint(4.0, 1.0, 0.0, 1)
            ];

            var curve = new NurbsCurve(degree, knotVector, controlPoints);

            using (var fs = new FileStream("test_curve.igs", FileMode.Create, FileAccess.Write))
            {
                await IGESExporter.ExportAsync([curve], fs);
            }
            Console.WriteLine($"IGES curve file 'test_curve.igs' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_curve.igs"), Is.True);
        }

        [Test]
        public async Task IgesReadTestA()
        {
            string resourcePath = Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "IO", "resources");
            string[] igesfile = ["test_output.igs", "test_outputB.igs"];
            (int degree_u,int degree_v,int cp_u,int cp_v,int k_u,int k_v)[] expect = [(3, 3, 4, 4, 8, 8),(3,3,5,5,9,9)];
            foreach (var file in igesfile)
            {
                string filepath = Path.Combine(resourcePath, file);
                Assert.That(File.Exists(filepath), Is.True, $"Required IGES file '{file}' does not exist in resources folder.");
                var reader = new StreamReader(filepath, Encoding.ASCII);
                var res = await IGESImporter.ImportAsync(reader);
                var surface = res.OfType<NurbsSurface>().FirstOrDefault();
                int index = igesfile.ToList().IndexOf(file);
                Assert.That(surface, Is.Not.Null, $"No NURBS surface found in IGES file '{file}'.");
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(surface.DegreeU, Is.EqualTo(expect[index].degree_u));
                    Assert.That(surface.DegreeV, Is.EqualTo(expect[index].degree_v));
                    Assert.That(surface.ControlPoints, Has.Length.EqualTo(expect[index].cp_u));
                
                    Assert.That(surface.ControlPoints[0], Has.Length.EqualTo(expect[index].cp_v));
                    Assert.That(surface.KnotVectorU.Knots, Has.Length.EqualTo(expect[index].k_u));
                    Assert.That(surface.KnotVectorV.Knots, Has.Length.EqualTo(expect[index].k_v));
                }
            }
        }

        [Test]
        public async Task IgesReadCurveTestA()
        {
            string resourcePath = Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "IO", "resources");
            string igesfile = "test_curve.igs";
            int expect_degree = 3;
            int expect_cp = 4;
            int expect_knot = 8;
            string filepath = Path.Combine(resourcePath, igesfile);
            Assert.That(File.Exists(filepath), Is.True, $"Required IGES file '{igesfile}' does not exist in resources folder.");
            var reader = new StreamReader(filepath, Encoding.ASCII);
            var res = await IGESImporter.ImportAsync(reader);
            var curve = res.OfType<NurbsCurve>().FirstOrDefault();
            Assert.That(curve, Is.Not.Null, $"No NURBS curve found in IGES file '{igesfile}'.");
            using (Assert.EnterMultipleScope())
            {
                Assert.That(curve.Degree, Is.EqualTo(expect_degree));
                Assert.That(curve.ControlPoints, Has.Length.EqualTo(expect_cp));
                Assert.That(curve.KnotVector.Knots, Has.Length.EqualTo(expect_knot));
            }
        }

        [Test]
        public async Task ExportSphereWireframe_ShouldCreateBMPFile()
        {
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            string filePath = "test_sphere_wireframe.bmp";

            // Clean up if file exists
            if (File.Exists(filePath))
                File.Delete(filePath);

            var campos = new Vector3Double(20, 20, 10);

            // Act
            await BMPExporter.ExportWireframeAsync(sphere, filePath, 800, 800, 20, 20, campos);

            // Assert
            Assert.That(File.Exists(filePath), Is.True);
            var fileInfo = new FileInfo(filePath);
            Assert.That(fileInfo.Length, Is.GreaterThan(0));

            // Verify BMP header
            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read);
            using var reader = new BinaryReader(fs);
            // Check BMP signature
            byte b1 = reader.ReadByte();
            byte b2 = reader.ReadByte();
            using (Assert.EnterMultipleScope())
            {
                Assert.That((char)b1, Is.EqualTo('B'));
                Assert.That((char)b2, Is.EqualTo('M'));
            }


        }

        [Test]
        public async Task ExportCircleWireframe_ShouldCreateBMPFile()
        {
            double radius = 3.0;
            var circle = PrimitiveFactory.CreateCircle(radius);
            string filePath = "test_circle_wireframe.bmp";

            // Clean up if file exists
            if (File.Exists(filePath))
                File.Delete(filePath);

            var campos = new Vector3Double(10, 10, 10);

            // Act
            await BMPExporter.ExportWireframeAsync(circle, filePath, 800, 800, 100, campos);

            // Assert
            Assert.That(File.Exists(filePath), Is.True);
            var fileInfo = new FileInfo(filePath);
            Assert.That(fileInfo.Length, Is.GreaterThan(0));
        }

        [Test]
        public async Task ExportCylinderWireframe_ShouldCreateBMPFile()
        {
            double radius = 2.0;
            double height = 5.0;
            var cylinder = PrimitiveFactory.CreateCylinder(radius, height, false);
            string filePath = "test_cylinder_wireframe.bmp";

            // Clean up if file exists
            if (File.Exists(filePath))
                File.Delete(filePath);

            var campos = new Vector3Double(20, 20, 50);
            // Act - Export first surface (side of cylinder)
            await BMPExporter.ExportWireframeAsync(cylinder[0], filePath, 800, 800, 20, 20,campos);

            // Assert
            Assert.That(File.Exists(filePath), Is.True);
            var fileInfo = new FileInfo(filePath);
            Assert.That(fileInfo.Length, Is.GreaterThan(0));

        }

        [Test]
        public async Task ExportSphereWithFrontCamera_ShouldCreateBMPFile()
        {
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            string filePath = "test_sphere_front_camera.bmp";

            // Clean up if file exists
            if (File.Exists(filePath))
                File.Delete(filePath);

            // Act - Export from front view
            await BMPExporter.ExportWireframeAsync(
                sphere,
                filePath,
                800, 800, 20, 20,
                cameraPosition: new Vector3Double(0, -15, 0)
            );

            // Assert
            Assert.That(File.Exists(filePath), Is.True);
            var fileInfo = new FileInfo(filePath);
            Assert.That(fileInfo.Length, Is.GreaterThan(0));

        }

        [Test]
        public async Task ExportSphereWithIsometricCamera_ShouldCreateBMPFile()
        {
            // Arrange
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            string filePath = "test_sphere_isometric_camera.bmp";

            // Clean up if file exists
            if (File.Exists(filePath))
                File.Delete(filePath);

            // Act - Export from isometric view
            await BMPExporter.ExportWireframeAsync(
                sphere,
                filePath,
                800, 800, 20, 20,
                cameraPosition: new Vector3Double(10, 10, 10)
            );

            // Assert
            Assert.That(File.Exists(filePath), Is.True);
            var fileInfo = new FileInfo(filePath);
            Assert.That(fileInfo.Length, Is.GreaterThan(0));

        }
    }
}
