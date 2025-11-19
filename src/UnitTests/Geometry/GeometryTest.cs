using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Geometry.Primitive;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.IO.IGES;

namespace UnitTests.Geometry
{
    [TestFixture]
    internal class GeometryTest
    {
        [Test]
        public void TestNurbsCurve()
        {
            
        }

        [Test]
        public void TestNurbsSurface()
        {

        }

        [Test]
        public void PrimitiveTestCircle()
        {
            double Radius = 5.0;
            int degree = 2;
            int numControlPoints = 9;
            int expectedKnotCount = degree + numControlPoints + 1;
            Assert.Throws<ArgumentOutOfRangeException>(() =>
            {
                PrimitiveFactory.CreateCircle(0);
            });
            
            Assert.Throws<ArgumentOutOfRangeException>(() => {
                PrimitiveFactory.CreateCircle(-1);
            });

            var circle2 = PrimitiveFactory.CreateCircle(Radius);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(circle2.Degree, Is.EqualTo(degree));
                Assert.That(circle2.ControlPoints, Has.Length.EqualTo(numControlPoints));
                Assert.That(circle2.KnotVector.Knots, Has.Length.EqualTo(expectedKnotCount));
            }
            for(int i = 0; i <= 100; i++)
            {
                double u = i / 100.0;
                var pt = circle2.GetPos(u);
                double dist = Math.Sqrt(pt.X * pt.X + pt.Y * pt.Y);
                Assert.That(dist, Is.EqualTo(Radius).Within(1e-6));
            }
        }

        [Test]
        public void PrimitiveTestFace()
        {
            Vector3Double p00 = new Vector3Double(0, 0, 0);
            Vector3Double p01 = new Vector3Double(10, 0, 5);
            Vector3Double p10 = new Vector3Double(0, 10, 0);
            Vector3Double p11 = new Vector3Double(10, 10, 5);
            var face = PrimitiveFactory.CreateFace(p00, p01, p10, p11);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(face.DegreeU, Is.EqualTo(1));
                Assert.That(face.DegreeV, Is.EqualTo(1));
                Assert.That(face.ControlPoints, Has.Length.EqualTo(2));
                Assert.That(face.ControlPoints[0], Has.Length.EqualTo(2));
                Assert.That(face.ControlPoints[1], Has.Length.EqualTo(2));
                Assert.That(face.KnotVectorU.Knots, Has.Length.EqualTo(4));
                Assert.That(face.KnotVectorV.Knots, Has.Length.EqualTo(4));
                var pos = SurfaceEvaluator.Evaluate(face, 0.5, 0.5);
                Assert.That(pos.X, Is.EqualTo(5).Within(1e-6));
                Assert.That(pos.Y, Is.EqualTo(5).Within(1e-6));
                Assert.That(pos.Z, Is.EqualTo(2.5).Within(1e-6));
            }
        }

        [Test]
        public void PrimitiveTestBox()
        {
            var box = PrimitiveFactory.CreateBox(10, 20, 30);
　          Assert.That(box.Length, Is.EqualTo(6));

            foreach(var face in box)
            {
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(face.DegreeU, Is.EqualTo(1));
                    Assert.That(face.DegreeV, Is.EqualTo(1));
                    Assert.That(face.ControlPoints, Has.Length.EqualTo(2));
                    Assert.That(face.ControlPoints[0], Has.Length.EqualTo(2));
                    Assert.That(face.ControlPoints[1], Has.Length.EqualTo(2));
                    Assert.That(face.KnotVectorU.Knots, Has.Length.EqualTo(4));
                    Assert.That(face.KnotVectorV.Knots, Has.Length.EqualTo(4));
                    
                }
            }
            TestOutputIGES(box.ToList());


        }

        private void TestOutputIGES(List<NurbsSurface> surface, string filePath= "PrimitiveTestFace.igs")
        {
            using var stream = new FileStream(filePath, FileMode.Create, FileAccess.Write);
            var success = IGESExporter.ExportAsync(surface, stream);
            
        }
    }
}
