using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NUnit.Framework;
using NUnit.Framework.Legacy;

namespace UnitTests.Core
{

    [TestFixture]
    public class CoreTest
    {
        [SetUp]
        public void Setup()
        {
        }

        [Test]
        public void Vector4DoubleCheck()
        {
            Vector4Double v = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Assert.That(1.0, Is.EqualTo(v.X));
            Assert.That(2.0, Is.EqualTo(v.Y));
            Assert.That(3.0, Is.EqualTo(v.Z));
            Assert.That(4.0, Is.EqualTo(v.W));
        }

        [Test]
        public void Vector3DoubleCheck()
        {
            Vector3Double v = new Vector3Double(1.0, 2.0, 3.0);
            Assert.That(1.0, Is.EqualTo(v.X));
            Assert.That(2.0, Is.EqualTo(v.Y));
            Assert.That(3.0, Is.EqualTo(v.Z));
        }

        [Test]
        public void VectorOperationCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(4.0, 5.0, 6.0);
            Vector3Double v3 = v1 + v2;
            Assert.That(5.0, Is.EqualTo(v3.X));
            Assert.That(7.0, Is.EqualTo(v3.Y));
            Assert.That(9.0, Is.EqualTo(v3.Z));
            Vector4Double w1 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w2 = new Vector4Double(5.0, 6.0, 7.0, 8.0);
            Vector4Double w3 = w1 + w2;
            Assert.That(6.0, Is.EqualTo(w3.X));
            Assert.That(8.0, Is.EqualTo(w3.Y));
            Assert.That(10.0, Is.EqualTo(w3.Z));
            Assert.That(12.0, Is.EqualTo(w3.W));
            // Scalar multiplication
            Vector3Double v4 = v1 * 2.0;
            Assert.That(2.0, Is.EqualTo(v4.X));
            Assert.That(4.0, Is.EqualTo(v4.Y));
            Assert.That(6.0, Is.EqualTo(v4.Z));
            Vector4Double w4 = 3.0 * w1;
            Assert.That(3.0, Is.EqualTo(w4.X));
            Assert.That(6.0, Is.EqualTo(w4.Y));
            Assert.That(9.0, Is.EqualTo(w4.Z));
            Assert.That(12.0, Is.EqualTo(w4.W));

        }



        [Test]
        public void ControlPointCheck()
        {
            ControlPoint cp = new ControlPoint(1.0, 2.0, 3.0, 4.0);
            Assert.That(1.0, Is.EqualTo(cp.Position.X));
            Assert.That(2.0, Is.EqualTo(cp.Position.Y));
            Assert.That(3.0, Is.EqualTo(cp.Position.Z));
            Assert.That(4.0, Is.EqualTo(cp.Weight));
            Vector4Double hp = cp.HomogeneousPosition;
            Assert.That(4.0, Is.EqualTo(hp.X));
            Assert.That(8.0, Is.EqualTo(hp.Y));
            Assert.That(12.0, Is.EqualTo(hp.Z));
            Assert.That(4.0, Is.EqualTo(hp.W));
        }

        [Test]
        public void KnotVectorCheck()
        {
            double[] knots = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
            KnotVector kv = new KnotVector(knots);
            Assert.That(6, Is.EqualTo(kv.Knots.Length));
            Assert.That(0.0, Is.EqualTo(kv.Knots[0]));
            Assert.That(1.0, Is.EqualTo(kv.Knots[5]));

            KnotVector uniformKv = KnotVector.GetUniformKnot(6);
            Assert.That(6, Is.EqualTo(uniformKv.Knots.Length));
            Assert.That(0.0, Is.EqualTo(uniformKv.Knots[0]));
            Assert.That(0.2, Is.EqualTo(uniformKv.Knots[1]));
            Assert.That(0.4, Is.EqualTo(uniformKv.Knots[2]));
            Assert.That(0.6, Is.EqualTo(uniformKv.Knots[3]));
            Assert.That(0.8, Is.EqualTo(uniformKv.Knots[4]));
            Assert.That(1.0, Is.EqualTo(uniformKv.Knots[5]));


        }
        [Test]
        public void Vector3DoubleDistanceCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(4.0, 6.0, 3.0);
            double distance = v1.DistanceTo(v2);
            // sqrt((4-1)^2 + (6-2)^2 + (3-3)^2) = sqrt(9 + 16 + 0) = sqrt(25) = 5
            Assert.That(distance, Is.EqualTo(5.0));
        }

        [Test]
        public void Vector3DoubleDotProductCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(4.0, 5.0, 6.0);
            double dot = Vector3Double.Dot(v1, v2);
            // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
            Assert.That(dot, Is.EqualTo(32.0));

            v1 = new Vector3Double(0, 0, 0);
            v2 = new Vector3Double(0, 0, 0);
            dot = Vector3Double.Dot(v1, v2);
            // 0*0 + 0*0 + 0*0 = 0
            Assert.That(dot, Is.EqualTo(0.0));

            v1 = new Vector3Double(-1, -3.5, -5.5);
            v2 = new Vector3Double(2, -2, 2);
            dot = Vector3Double.Dot(v1, v2);
            // -1*2 + -3.5*-2 + -5.5*2 = -2 + 7 - 11 = -6
            Assert.That(dot, Is.EqualTo(-6.0));
        }

        [Test]
        public void Vector3DoubleCrossProductCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(4.0, 5.0, 6.0);
            Vector3Double cross = Vector3Double.Cross(v1, v2);
            // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
            Assert.That(cross.X, Is.EqualTo(-3.0));
            Assert.That(cross.Y, Is.EqualTo(6.0));
            Assert.That(cross.Z, Is.EqualTo(-3.0));

            v1 = new Vector3Double(0, 0, 0);
            v2 = new Vector3Double(0, 0, 0);
            cross = Vector3Double.Cross(v1, v2);
            // (0, 0, 0)
            Assert.That(cross.X, Is.EqualTo(0.0));
            Assert.That(cross.Y, Is.EqualTo(0.0));
            Assert.That(cross.Z, Is.EqualTo(0.0));

        }
    }
}
