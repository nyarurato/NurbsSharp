using System;
using System.Collections.Generic;

using NUnit.Framework;
using NUnit.Framework.Legacy;

using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;


namespace UnitTests.Evaluation
{
    [TestFixture]
    public class VolumeEvaluateTest
    {
      

        [Test]
        public void NurbsVolumeTestA()
        {
            // Bilinear NURBS volue (degree 1 in both directions)
            int degreeU = 1;
            int degreeV = 1;
            int degreeW = 1;

            double[] knotsU = { 0, 0, 1, 1 };
            double[] knotsV = { 0, 0, 1, 1 };
            double[] knotsW = { 0, 0, 1, 1 };
            ControlPoint[][][] controlPoints = new ControlPoint[2][][];
            controlPoints[0] = new ControlPoint[2][];
            controlPoints[0][0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1)
            };
            controlPoints[0][1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1)
            };
            controlPoints[1] = new ControlPoint[2][];
            controlPoints[1][0] = new ControlPoint[] {
                new ControlPoint(0.0, 0.0, 1.0, 1),
                new ControlPoint(1.0, 0.0, 2.0, 1)
            };
            controlPoints[1][1] = new ControlPoint[] {
                new ControlPoint(0.0, 1.0, 1.5, 1),
                new ControlPoint(1.0, 1.0, 2.5, 1)
            };
            var volume = new NurbsVolume(degreeU, degreeV, degreeW, new KnotVector(knotsU), new KnotVector(knotsV), new KnotVector(knotsW), controlPoints);

            var samples = new (double u, double v, double w, Vector3Double expected)[] {
                (0.000, 0.000, 0.000, new Vector3Double(0.000000, 0.000000, 0.000000)),
                (1.000, 0.000, 0.000, new Vector3Double(0.000000, 0.000000, 1.000000)),
                (0.000, 1.000, 0.000, new Vector3Double(0.000000, 1.000000, 0.500000)),
                (1.000, 1.000, 0.000, new Vector3Double(0.000000, 1.000000, 1.500000)),
                (0.000, 0.000, 1.000, new Vector3Double(1.000000, 0.000000, 1.000000)),
                (1.000, 0.000, 1.000, new Vector3Double(1.000000, 0.000000, 2.000000)),
                (0.000, 1.000, 1.000, new Vector3Double(1.000000, 1.000000, 1.500000)),
                (1.000, 1.000, 1.000, new Vector3Double(1.000000, 1.000000, 2.500000)),
                (0.500, 0.500, 0.500, new Vector3Double(0.500000, 0.50000, 1.25000))
            };

            foreach (var (u, v, w, expected) in samples)
            {
                var pt = VolumeEvaluator.Evaluate(volume, u, v, w);
                Console.WriteLine($"Evaluating at u={u}, v={v}, w={w} expected={expected} pt={pt}");
                Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
            }
        }

        
    }


}