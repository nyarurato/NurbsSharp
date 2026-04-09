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
        private static NurbsVolume BuildLinearUnitCube()
        {
            int degree = 1;
            double[] knots = [0, 0, 1, 1];
            ControlPoint[][][] controlPoints = new ControlPoint[2][][];
            controlPoints[0] = new ControlPoint[2][];
            controlPoints[0][0] = [
                new ControlPoint(0.0, 0.0, 0.0, 1),
                new ControlPoint(1.0, 0.0, 1.0, 1)
            ];
            controlPoints[0][1] = [
                new ControlPoint(0.0, 1.0, 0.5, 1),
                new ControlPoint(1.0, 1.0, 1.5, 1)
            ];
            controlPoints[1] = new ControlPoint[2][];
            controlPoints[1][0] = [
                new ControlPoint(0.0, 0.0, 1.0, 1),
                new ControlPoint(1.0, 0.0, 2.0, 1)
            ];
            controlPoints[1][1] = [
                new ControlPoint(0.0, 1.0, 1.5, 1),
                new ControlPoint(1.0, 1.0, 2.5, 1)
            ];
            return new NurbsVolume(
                degree, degree, degree,
                new KnotVector(knots, degree),
                new KnotVector(knots, degree),
                new KnotVector(knots, degree),
                controlPoints);
        }

        [Test]
        public void NurbsVolumeTestA()
        {
            // Bilinear NURBS volume (degree 1 in all directions)
            var volume = BuildLinearUnitCube();

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
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(expected.X, Is.EqualTo(pt.x).Within(0.000001));
                    Assert.That(expected.Y, Is.EqualTo(pt.y).Within(0.000001));
                    Assert.That(expected.Z, Is.EqualTo(pt.z).Within(0.000001));
                }
            }
        }

        /// <summary>
        /// A uniform grid axis-aligned NURBS volume (degree 1) maps parameters linearly to positions.
        /// </summary>
        [Test]
        public void NurbsVolume_LinearAxisAligned_MapsParametersToPositions()
        {
            // Simple axis-aligned unit cube: P(u,v,w) = (w, v, u) for u,v,w in [0,1]
            int degree = 1;
            double[] knots = [0, 0, 1, 1];

            ControlPoint[][][] cps = new ControlPoint[2][][];
            for (int i = 0; i < 2; i++)
            {
                cps[i] = new ControlPoint[2][];
                for (int j = 0; j < 2; j++)
                {
                    cps[i][j] = new ControlPoint[2];
                    for (int k = 0; k < 2; k++)
                    {
                        cps[i][j][k] = new ControlPoint(k, j, i, 1.0);
                    }
                }
            }

            var volume = new NurbsVolume(
                degree, degree, degree,
                new KnotVector(knots, degree),
                new KnotVector(knots, degree),
                new KnotVector(knots, degree),
                cps);

            // Evaluate at corners and center
            var corners = new (double u, double v, double w)[]
            {
                (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1),
                (1, 1, 1), (0.5, 0.5, 0.5)
            };

            foreach (var (u, v, w) in corners)
            {
                var pt = VolumeEvaluator.Evaluate(volume, u, v, w);
                using (Assert.EnterMultipleScope())
                {
                    Assert.That(pt.x, Is.EqualTo(w).Within(1e-10));
                    Assert.That(pt.y, Is.EqualTo(v).Within(1e-10));
                    Assert.That(pt.z, Is.EqualTo(u).Within(1e-10));
                }
            }
        }

        /// <summary>
        /// NURBS volume bounding box encompasses all control points.
        /// </summary>
        [Test]
        public void NurbsVolume_BoundingBox_ContainsAllControlPoints()
        {
            var volume = BuildLinearUnitCube();
            var bbox = volume.BoundingBox;

            // All evaluated points must lie within the bounding box (with tolerance)
            for (int si = 0; si <= 4; si++)
                for (int sj = 0; sj <= 4; sj++)
                    for (int sk = 0; sk <= 4; sk++)
                    {
                        double u = si / 4.0;
                        double v = sj / 4.0;
                        double w = sk / 4.0;
                        var pt = VolumeEvaluator.Evaluate(volume, u, v, w);

                        Assert.That(pt.x, Is.GreaterThanOrEqualTo(bbox.Min.X - 1e-9));
                        Assert.That(pt.x, Is.LessThanOrEqualTo(bbox.Max.X + 1e-9));
                        Assert.That(pt.y, Is.GreaterThanOrEqualTo(bbox.Min.Y - 1e-9));
                        Assert.That(pt.y, Is.LessThanOrEqualTo(bbox.Max.Y + 1e-9));
                        Assert.That(pt.z, Is.GreaterThanOrEqualTo(bbox.Min.Z - 1e-9));
                        Assert.That(pt.z, Is.LessThanOrEqualTo(bbox.Max.Z + 1e-9));
                    }
        }

        /// <summary>
        /// NurbsVolume throws on null argument to Evaluate.
        /// </summary>
        [Test]
        public void NurbsVolume_Evaluate_NullThrows()
        {
            Assert.Throws<ArgumentNullException>(() =>
            {
                VolumeEvaluator.Evaluate(null!, 0.5, 0.5, 0.5);
            });
        }

        /// <summary>
        /// NurbsVolume constructor validates knot-vector vs. control-point dimensions.
        /// </summary>
        [Test]
        public void NurbsVolume_Constructor_InvalidKnotLengthThrows()
        {
            int degree = 1;
            double[] validKnots = [0, 0, 1, 1];
            double[] badKnots   = [0, 0, 0.5, 1, 1]; // length 5 instead of 4

            ControlPoint[][][] cps = new ControlPoint[2][][];
            for (int i = 0; i < 2; i++)
            {
                cps[i] = new ControlPoint[2][];
                for (int j = 0; j < 2; j++)
                    cps[i][j] = [new ControlPoint(0, 0, 0, 1), new ControlPoint(1, 0, 0, 1)];
            }

            Assert.Throws<InvalidOperationException>(() =>
            {
                _ = new NurbsVolume(
                    degree, degree, degree,
                    new KnotVector(badKnots, degree),
                    new KnotVector(validKnots, degree),
                    new KnotVector(validKnots, degree),
                    cps);
            });
        }
    }


}