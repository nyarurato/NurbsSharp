using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.IO;
using NurbsSharp.IO.IGES;
using NurbsSharp.Tesselation;

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
            KnotVector knotVectorU = new KnotVector(new double[] { 0, 0, 1, 1 });
            KnotVector knotVectorV = new KnotVector(new double[] { 0, 0, 1, 1 });
            ControlPoint[][] controlPoints = new ControlPoint[][]
            {
                new ControlPoint[]
                {
                    new ControlPoint(0, 0, 0),
                    new ControlPoint(0, 1, 0)
                },
                new ControlPoint[]
                {
                    new ControlPoint(1, 0, 1),
                    new ControlPoint(1, 1, 1)
                }
            };

            string estimate_str = "v 0 0 0\n" +
                                 "v 0 1 0\n" +
                                 "v 1 0 1\n" +
                                 "v 1 1 1\n" +
                                 "f 1 3 4\n" +
                                 "f 1 4 2";

            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 2, 2);

            using (MemoryStream ms = new MemoryStream())
            {
                await OBJExporter.ExportAsync(mesh, ms);
                ms.Seek(0, SeekOrigin.Begin);
                using (StreamReader reader = new StreamReader(ms))
                {
                    string objData = reader.ReadToEnd();

                    Assert.That(
                        objData.Replace("\r\n", "\n").Trim(['\r', '\n', ' ']),
                        Is.EqualTo(estimate_str.Trim(['\r', '\n', ' ']))
                    );
                }
            }
        }

        [Test]
        public async Task OutputObjFile()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;

            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

            KnotVector knotVectorU = new KnotVector(knotsU);
            KnotVector knotVectorV = new KnotVector(knotsV);

            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1),
                new ControlPoint(2.0, 0.0, 3.0, 1),
                new ControlPoint(3.0, 0.0, 3.0, 1)

            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1),
                new ControlPoint(3.0, 1.0, 3.0, 1)

            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(3.0, 1.0, 0.5, 1),
                new ControlPoint(3.0, 1.0, 1.5, 1),
                new ControlPoint(3.0, 1.0, 5.0, 1),
                new ControlPoint(3.0, 1.0, 6.0, 1)

            };
            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(3.0, 2.0, 0.5, 1),
                new ControlPoint(3.0, 2.0, 1.5, 1),
                new ControlPoint(3.0, 2.0, 5.0, 1),
                new ControlPoint(3.0, 2.0, 7.0, 1)

            };
            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 20, 20);
            using (FileStream fs = new FileStream("test_output.obj", FileMode.Create, FileAccess.Write))
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

            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

            KnotVector knotVectorU = new KnotVector(knotsU);
            KnotVector knotVectorV = new KnotVector(knotsV);

            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1),
                new ControlPoint(2.0, 0.0, 3.0, 1),
                new ControlPoint(3.0, 0.0, 3.0, 1)

            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1),
                new ControlPoint(3.0, 1.0, 3.0, 1)

            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(3.0, 1.0, 0.5, 1),
                new ControlPoint(3.0, 1.0, 1.5, 1),
                new ControlPoint(3.0, 1.0, 5.0, 1),
                new ControlPoint(3.0, 1.0, 6.0, 1)

            };
            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(3.0, 2.0, 0.5, 1),
                new ControlPoint(3.0, 2.0, 1.5, 1),
                new ControlPoint(3.0, 2.0, 5.0, 1),
                new ControlPoint(3.0, 2.0, 7.0, 1)

            };
            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);

            using (FileStream fs = new FileStream("test_output_nurbs.obj", FileMode.Create, FileAccess.Write))
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

            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

            KnotVector knotVectorU = new KnotVector(knotsU);
            KnotVector knotVectorV = new KnotVector(knotsV);

            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1),
                new ControlPoint(2.0, 0.0, 3.0, 1),
                new ControlPoint(3.0, 0.0, 3.0, 1)

            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1),
                new ControlPoint(3.0, 1.0, 3.0, 1)

            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(3.0, 1.0, 0.5, 1),
                new ControlPoint(3.0, 1.0, 1.5, 1),
                new ControlPoint(3.0, 1.0, 5.0, 1),
                new ControlPoint(3.0, 1.0, 6.0, 1)

            };
            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(3.0, 2.0, 0.5, 1),
                new ControlPoint(3.0, 2.0, 1.5, 1),
                new ControlPoint(3.0, 2.0, 5.0, 1),
                new ControlPoint(3.0, 2.0, 7.0, 1)

            };
            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 20, 20);
            using (FileStream fs = new FileStream("test_output.stl", FileMode.Create, FileAccess.Write))
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
            double[] knotsU = { 0, 0, 1, 1 };
            double[] knotsV = { 0, 0, 1, 1 };
            ControlPoint[][] controlPoints = new ControlPoint[2][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 0.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 1.5, 1),
                new ControlPoint(1.0, 0.0, 1.5, 1)
            };
            KnotVector knotVectorU = new KnotVector(knotsU);
            KnotVector knotVectorV = new KnotVector(knotsV);
            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 100, 100);
            using (FileStream fs = new FileStream("test_outputB.stl", FileMode.Create, FileAccess.Write))
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

            double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

            KnotVector knotVectorU = new KnotVector(knotsU);
            KnotVector knotVectorV = new KnotVector(knotsV);

            ControlPoint[][] controlPoints = new ControlPoint[4][];
            controlPoints[0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1),
                new ControlPoint(2.0, 0.0, 3.0, 1),
                new ControlPoint(3.0, 0.0, 3.0, 1)

            };
            controlPoints[1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1),
                new ControlPoint(2.0, 1.0, 4.0, 1),
                new ControlPoint(3.0, 1.0, 3.0, 1)

            };
            controlPoints[2] = new ControlPoint[] {
                new ControlPoint(3.0, 1.0, 0.5, 1),
                new ControlPoint(3.0, 1.0, 1.5, 1),
                new ControlPoint(3.0, 1.0, 5.0, 1),
                new ControlPoint(3.0, 1.0, 6.0, 1)

            };
            controlPoints[3] = new ControlPoint[] {
                new ControlPoint(3.0, 2.0, 0.5, 1),
                new ControlPoint(3.0, 2.0, 1.5, 1),
                new ControlPoint(3.0, 2.0, 5.0, 1),
                new ControlPoint(3.0, 2.0, 7.0, 1)

            };
            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);

            using (FileStream fs = new FileStream("test_output.igs", FileMode.Create, FileAccess.Write))
            {
                await IGESExporter.ExportAsync(nurbsSurface, fs);
            }
            Console.WriteLine($"STL file 'test_output.igs' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_output.igs"), Is.True);
        }
    


    [Test]
        public async Task OutputIgesFileB()
        {
            // Create a simple NURBS surface
            int degreeU = 3;
            int degreeV = 3;
            double[] knotsU = { 0, 0, 0, 0, 0.5,1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 0.5,1, 1, 1, 1 };
            KnotVector knotVectorU = new KnotVector(knotsU);
            KnotVector knotVectorV = new KnotVector(knotsV);
            ControlPoint[][] controlPoints = new ControlPoint[5][];
            controlPoints[0] = new ControlPoint[] {
            new ControlPoint(0.0, 0.0, 0.0, 1),
            new ControlPoint(1.0, 0.0, 0.0, 1),
            new ControlPoint(2.0, 0.0, 0.0, 1),
            new ControlPoint(3.0, 0.0, 0.0, 1),
            new ControlPoint(4.0, 0.0, 0.0, 1)
            };
            controlPoints[1] = new ControlPoint[] {
            new ControlPoint(0.0, 1.0, 0.5, 1),
            new ControlPoint(1.0, 1.0, -1.5, 1),
            new ControlPoint(2.0, 1.0, 4.0, 1),
            new ControlPoint(3.0, 1.0, -3.0, 1),
            new ControlPoint(4.0, 1.0, 0.5, 1)
            };
            controlPoints[2] = new ControlPoint[] {
            new ControlPoint(0.0, 2.0, 1.5, 1),
            new ControlPoint(1.0, 2.0, 2.5, 1),
            new ControlPoint(2.0, 2.0, 3.5, 0.7),
            new ControlPoint(3.0, 2.0, 3.0, 1),
            new ControlPoint(4.0, 2.0, 0.0, 1)
            };
            controlPoints[3] = new ControlPoint[] {
            new ControlPoint(0.0, 3.0, 0.5, 1),
            new ControlPoint(1.5, 3.0, -1.5, 1),
            new ControlPoint(2.5, 3.0, 2.0 ,1),
            new ControlPoint(3.5, 3.0, -1.5, 1),
            new ControlPoint(4.5, 3.0, -1.0, 1)
            };
            controlPoints[4] = new ControlPoint[] {
            new ControlPoint(0.0, 4.0, 0.5, 1),
            new ControlPoint(1.0, 4.0, 0.5, 1),
            new ControlPoint(2.0, 4.0, 0.0, 1),
            new ControlPoint(3.0, 4.0, 0.0, 1),
            new ControlPoint(4.0, 4.0, 0.0, 1) 
            };
            NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            using (FileStream fs = new FileStream("test_outputB.igs", FileMode.Create, FileAccess.Write))
            {
                await IGESExporter.ExportAsync(nurbsSurface, fs);
            }
            Console.WriteLine($"STL file 'test_outputB.igs' has been created. At Current :{Directory.GetCurrentDirectory()}");
            Assert.That(File.Exists("test_outputB.igs"), Is.True);

        }
    }
}
