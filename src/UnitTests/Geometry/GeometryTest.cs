using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.IO.IGES;
using NurbsSharp.IO;
using NurbsSharp.Tesselation;

namespace UnitTests.Geometry
{
    [TestFixture]
    internal class GeometryTest
    {
        [Test]
        public void TestNurbsCurve()
        {
            //translate test
            int degree = 2;
            double[] knots = [0, 0, 0, 0.5, 1, 1, 1];
            var knotVector = new KnotVector(knots,degree);
            ControlPoint[] controlPoints =
            [
                new ControlPoint(0, 0, 0),
                new ControlPoint(5, 10, 0),
                new ControlPoint(10, 0, 0),
                new ControlPoint(15, -10, 0),
            ];
            ControlPoint[] originalControlPoints = [.. controlPoints.Select(cp => new ControlPoint(cp.Position.X, cp.Position.Y, cp.Position.Z))];
            var nurbsCurve = new NurbsCurve(degree, knotVector, controlPoints);
            var move = new Vector3Double(1, -2.5, 3);
            nurbsCurve.Translate(move);
            for(int i = 0; i < controlPoints.Length; i++)
            {
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(nurbsCurve.ControlPoints[i].Position.X, Is.EqualTo(originalControlPoints[i].Position.X + move.X).Within(1e-6));
                    Assert.That(nurbsCurve.ControlPoints[i].Position.Y, Is.EqualTo(originalControlPoints[i].Position.Y + move.Y).Within(1e-6));
                    Assert.That(nurbsCurve.ControlPoints[i].Position.Z, Is.EqualTo(originalControlPoints[i].Position.Z + move.Z).Within(1e-6));
                }
            }

        }

        [Test]
        public void TestNurbsSurface()
        {
            //translate test
            int degreeU = 1;
            int degreeV = 1;
            ControlPoint[][] controlPoints =
            [
                [
                    new ControlPoint(0,0,0),
                    new ControlPoint(10,0,5),
                ],
                [
                    new ControlPoint(0,10,0),
                    new ControlPoint(10,10,5),
                ]
            ];
            ControlPoint[][] originalControlPoints = controlPoints
                .Select(row => row.Select(cp => new ControlPoint(cp.Position.X, cp.Position.Y, cp.Position.Z)).ToArray())
                .ToArray();
            KnotVector knotU = KnotVector.GetClampedKnot(degreeU, controlPoints.Length);
            KnotVector knotV = KnotVector.GetClampedKnot(degreeV, controlPoints[0].Length);

            var nurbsSurface = new NurbsSurface(degreeU, degreeV, knotU, knotV, controlPoints);
            var move = new Vector3Double(-2, 3.5, 1);

            nurbsSurface.Translate(move);
            for (int i = 0; i < controlPoints.Length; i++)
            {
                for (int j = 0; j < controlPoints[i].Length; j++)
                {
                    using (Assert.EnterMultipleScope())
                    {
                        Assert.That(nurbsSurface.ControlPoints[i][j].Position.X, Is.EqualTo(originalControlPoints[i][j].Position.X + move.X).Within(1e-6));
                        Assert.That(nurbsSurface.ControlPoints[i][j].Position.Y, Is.EqualTo(originalControlPoints[i][j].Position.Y + move.Y).Within(1e-6));
                        Assert.That(nurbsSurface.ControlPoints[i][j].Position.Z, Is.EqualTo(originalControlPoints[i][j].Position.Z + move.Z).Within(1e-6));
                    }
                }
            }
        }
    }
}
