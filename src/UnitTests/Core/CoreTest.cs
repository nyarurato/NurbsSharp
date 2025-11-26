using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NUnit.Framework;
using NUnit.Framework.Legacy;
using System.Numerics;

namespace UnitTests.Core
{

    [TestFixture]
    public class CoreTest
    {
        [SetUp]
        public void Setup()
        {
        }

        /*
         * 
         * KnotVector tests
         * 
         */


        [Test]
        public void KnotVectorCheck()
        {
            double[] knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
            KnotVector kv = new KnotVector(knots,2);
            Assert.That(kv.Knots, Has.Length.EqualTo(6));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(kv.Knots[0], Is.Zero);
                Assert.That(kv.Knots[5], Is.EqualTo(1.0));
            }

            KnotVector uniformKv = KnotVector.GetUniformKnot(6);
            double[] actual = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];
            Assert.That(uniformKv.Knots, Has.Length.EqualTo(6));
            Assert.That(uniformKv.Knots, Is.EqualTo(actual));

            //normalize test
            double[] knots2 = [1.0, 2.0, 3.0, 3.5, 4.0, 5.0];
            KnotVector kv2 = new KnotVector(knots2,0);
            actual = [0, 0.25, 0.5, 0.625, 0.75, 1.0];
            kv2.Normalize();
            Assert.That(kv2.Knots, Has.Length.EqualTo(6));
            Assert.That(kv2.Knots, Is.EqualTo(actual));

            // Validate test
            double[] invalidKnots = [0.0, 0.5, 0.3, 1.0];            
            Assert.Throws<ArgumentException>(() => new KnotVector(invalidKnots,1));

            // Clamped vector test
            int p = 2,Nctrp = 4;
            KnotVector clampedKv = KnotVector.GetClampedKnot(p,Nctrp,p+1);
            Assert.That(clampedKv.Knots, Has.Length.EqualTo(7));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(clampedKv.Knots[0], Is.Zero);
                Assert.That(clampedKv.Knots[1], Is.Zero);
                Assert.That(clampedKv.Knots[2], Is.Zero);
                Assert.That(clampedKv.Knots[3], Is.EqualTo(0.5));
                Assert.That(clampedKv.Knots[4], Is.EqualTo(1.0));
                Assert.That(clampedKv.Knots[5], Is.EqualTo(1.0));
                Assert.That(clampedKv.Knots[6], Is.EqualTo(1.0));
            }

            p = 3; Nctrp = 8;
            KnotVector clampedKv2 = KnotVector.GetClampedKnot(p, Nctrp);
            actual =[ 0.0, 0.0, 0.0, 0.0,0.2,0.4,0.6,0.8, 1.0, 1.0, 1.0, 1.0 ];//len 12 
            Assert.That(clampedKv2.Knots, Has.Length.EqualTo(12));
            Assert.That(clampedKv2.Knots, Is.EqualTo(actual));


            Assert.Throws<ArgumentOutOfRangeException>(() => KnotVector.GetClampedKnot(3,3,4));
            Assert.Throws<ArgumentOutOfRangeException>(() => KnotVector.GetClampedKnot(2,4,5));


        }

        /*
         * 
         * ControlPoint tests
         * 
         */

        [Test]
        public void ControlPointCheck()
        {
            ControlPoint cp = new ControlPoint(1.0, 2.0, 3.0, 4.0);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(cp.Position.X, Is.EqualTo(1.0));
                Assert.That(cp.Position.Y, Is.EqualTo(2.0));
                Assert.That(cp.Position.Z, Is.EqualTo(3.0));
                Assert.That(cp.Weight, Is.EqualTo(4.0));
            }
            Vector4Double hp = cp.HomogeneousPosition;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(hp.X, Is.EqualTo(4.0));
                Assert.That(hp.Y, Is.EqualTo(8.0));
                Assert.That(hp.Z, Is.EqualTo(12.0));
                Assert.That(hp.W, Is.EqualTo(4.0));
            }

            //translate by vector
            cp.Translate(new Vector3Double(1.0, 1.0, 1.0));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(cp.Position.X, Is.EqualTo(2.0));
                Assert.That(cp.Position.Y, Is.EqualTo(3.0));
                Assert.That(cp.Position.Z, Is.EqualTo(4.0));
                Assert.That(cp.Weight, Is.EqualTo(4.0));
            }
            cp = new ControlPoint(1.0, 2.0, 3.0, 4.0);
            cp.Translate(2, 2, 2);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(cp.Position.X, Is.EqualTo(3.0));
                Assert.That(cp.Position.Y, Is.EqualTo(4.0));
                Assert.That(cp.Position.Z, Is.EqualTo(5.0));
                Assert.That(cp.Weight, Is.EqualTo(4.0));
            }
        }

        /*
         * 
         * VectorDouble tests
         * 
         */


        [Test]
        public void Vector4DoubleCheck()
        {
            Vector4Double v = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v.X, Is.EqualTo(1.0));
                Assert.That(v.Y, Is.EqualTo(2.0));
                Assert.That(v.Z, Is.EqualTo(3.0));
                Assert.That(v.W, Is.EqualTo(4.0));
            }
            Vector4 vSys = v.ToVector4();
            Assert.That(vSys, Is.TypeOf(typeof(Vector4)));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(vSys.X, Is.EqualTo((float)v.X));
                Assert.That(vSys.Y, Is.EqualTo((float)v.Y));
                Assert.That(vSys.Z, Is.EqualTo((float)v.Z));
                Assert.That(vSys.W, Is.EqualTo((float)v.W));
            }
            Vector4 v2 = (Vector4)v;
            Assert.That(v2, Is.TypeOf(typeof(Vector4)));
        }

        [Test]
        public void Vector3DoubleCheck()
        {
            Vector3Double v = new Vector3Double(1.0, 2.0, 3.0);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v.X, Is.EqualTo(1.0));
                Assert.That(v.Y, Is.EqualTo(2.0));
                Assert.That(v.Z, Is.EqualTo(3.0));
            }

            Vector3 vSys = v.ToVector3();
            Assert.That(vSys, Is.TypeOf(typeof(Vector3)));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(vSys.X, Is.EqualTo((float)v.X));
                Assert.That(vSys.Y, Is.EqualTo((float)v.Y));
                Assert.That(vSys.Z, Is.EqualTo((float)v.Z));
            }
            Vector3 v2 = (Vector3)v;
            Assert.That(v2, Is.TypeOf(typeof(Vector3)));
        }

        [Test]
        public void VectorOperationCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(4.0, 5.0, 6.0);
            Vector3Double v3 = v1 + v2;
            Assert.That(v3, Is.EqualTo(new Vector3Double(5.0, 7.0, 9.0)));
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v3.X, Is.EqualTo(5.0));
                Assert.That(v3.Y, Is.EqualTo(7.0));
                Assert.That(v3.Z, Is.EqualTo(9.0));
            }
            Vector4Double w1 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w2 = new Vector4Double(5.0, 6.0, 7.0, 8.0);
            Vector4Double w3 = w1 + w2;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w3.X, Is.EqualTo(6.0));
                Assert.That(w3.Y, Is.EqualTo(8.0));
                Assert.That(w3.Z, Is.EqualTo(10.0));
                Assert.That(w3.W, Is.EqualTo(12.0));
            }
            // Scalar multiplication
            Vector3Double v4 = v1 * 2.0;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v4.X, Is.EqualTo(2.0));
                Assert.That(v4.Y, Is.EqualTo(4.0));
                Assert.That(v4.Z, Is.EqualTo(6.0));
            }
            v4 = 2.0 * v1;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v4.X, Is.EqualTo(2.0));
                Assert.That(v4.Y, Is.EqualTo(4.0));
                Assert.That(v4.Z, Is.EqualTo(6.0));
            }
            Vector4Double w4 = 3.0 * w1;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w4.X, Is.EqualTo(3.0));
                Assert.That(w4.Y, Is.EqualTo(6.0));
                Assert.That(w4.Z, Is.EqualTo(9.0));
                Assert.That(w4.W, Is.EqualTo(12.0));
            }
            w4 = w1 * 3.0;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w4.X, Is.EqualTo(3.0));
                Assert.That(w4.Y, Is.EqualTo(6.0));
                Assert.That(w4.Z, Is.EqualTo(9.0));
            }

            // div
            Vector3Double v5 = v2 / 2.0;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v5.X, Is.EqualTo(2.0));
                Assert.That(v5.Y, Is.EqualTo(2.5));
                Assert.That(v5.Z, Is.EqualTo(3.0));
            }
            //zero division check
            Assert.Throws<DivideByZeroException>(() => { var v6 = v2 / 0.0; });

            Vector4Double w5 = w2 / 2.0;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w5.X, Is.EqualTo(2.5));
                Assert.That(w5.Y, Is.EqualTo(3.0));
                Assert.That(w5.Z, Is.EqualTo(3.5));
                Assert.That(w5.W, Is.EqualTo(4.0));
            }
            //zero division check
            Assert.Throws<DivideByZeroException>(() => { var w6 =  w2 / 0.0; });

            Vector3Double v6 = -v1;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v6.X, Is.EqualTo(-1.0));
                Assert.That(v6.Y, Is.EqualTo(-2.0));
                Assert.That(v6.Z, Is.EqualTo(-3.0));
            }

            Vector4Double w6 = -w1;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w6.X, Is.EqualTo(-1.0));
                Assert.That(w6.Y, Is.EqualTo(-2.0));
                Assert.That(w6.Z, Is.EqualTo(-3.0));
                Assert.That(w6.W, Is.EqualTo(-4.0));
            }

            Vector3 nv = v1.ToVector3();
            Assert.That(nv, Is.TypeOf(typeof(Vector3)));
            var nv2 = (Vector3)v1;
            Assert.That(nv2, Is.TypeOf(typeof(Vector3)));

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
            Assert.That(dot, Is.Zero);

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
            using (Assert.EnterMultipleScope())
            {
                // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
                Assert.That(cross.X, Is.EqualTo(-3.0));
                Assert.That(cross.Y, Is.EqualTo(6.0));
                Assert.That(cross.Z, Is.EqualTo(-3.0));
            }

            v1 = new Vector3Double(0, 0, 0);
            v2 = new Vector3Double(0, 0, 0);
            cross = Vector3Double.Cross(v1, v2);
            using (Assert.EnterMultipleScope())
            {
                // (0, 0, 0)
                Assert.That(cross.X, Is.Zero);
                Assert.That(cross.Y, Is.Zero);
                Assert.That(cross.Z, Is.Zero);
            }

        }

        [Test]
        public void Vector3DoublMagnitudeCheck()
        {
            Vector3Double v = new Vector3Double(3.0, 4.0, 12.0);
            double length = v.magnitude;
            // sqrt(3^2 + 4^2 + 12^2) = sqrt(9 + 16 + 144) = sqrt(169) = 13
            Assert.That(length, Is.EqualTo(13.0));

            v = new Vector3Double(0.0, 0.0, 0.0);
            length = v.magnitude;
            // sqrt(0^2 + 0^2 + 0^2) = sqrt(0) = 0
            Assert.That(length, Is.Zero);
        }

        [Test]
        public void Vector3DoubleNormalizationCheck()
        {
            Vector3Double v = new Vector3Double(3.0, 4.0, 0.0);
            Vector3Double normalized = v.normalized;
            // Length should be 1
            double length = Math.Sqrt(normalized.X * normalized.X + normalized.Y * normalized.Y + normalized.Z * normalized.Z);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(length, Is.EqualTo(1.0).Within(0.000001));
                // Components should be (0.6, 0.8, 0.0)
                Assert.That(normalized.X, Is.EqualTo(0.6).Within(0.000001));
                Assert.That(normalized.Y, Is.EqualTo(0.8).Within(0.000001));
                Assert.That(normalized.Z, Is.Zero.Within(0.000001));
            }

            v = new Vector3Double(0.0, 0.0, 0.0);
            Assert.Throws<InvalidOperationException>(() => _ =v.normalized);
        }

        [Test]
        public void Vector3DoubleZeroCheck()
        {
            Vector3Double zero = Vector3Double.Zero;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(zero.X, Is.Zero);
                Assert.That(zero.Y, Is.Zero);
                Assert.That(zero.Z, Is.Zero);
            }
        }

        [Test]
        public void Vector3DoubleSubtractionOperatorCheck()
        {
            Vector3Double v1 = new Vector3Double(5.0, 7.0, 9.0);
            Vector3Double v2 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v3 = v1 - v2;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v3.X, Is.EqualTo(4.0));
                Assert.That(v3.Y, Is.EqualTo(5.0));
                Assert.That(v3.Z, Is.EqualTo(6.0));
            }

            v1 = new Vector3Double(0.0, 0.0, 0.0);
            v2 = new Vector3Double(1.0, 2.0, 3.0);
            v3 = v1 - v2;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(v3.X, Is.EqualTo(-1.0));
                Assert.That(v3.Y, Is.EqualTo(-2.0));
                Assert.That(v3.Z, Is.EqualTo(-3.0));
            }
        }

        [Test]
        public void Vector3DoubleEqualityOperatorCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v3 = new Vector3Double(1.0, 2.0, 4.0);

            using (Assert.EnterMultipleScope())
            {
                // == operator
                bool eq = v1 == v2;
                Assert.That(eq, Is.True);

                eq = v1 == v3;
                Assert.That(eq, Is.False);

                // != operator
                eq = v1 != v3;
                Assert.That(eq, Is.True);

                eq = v1 != v2;
                Assert.That(eq, Is.False);
            }

            // Edge case: zero vectors
            Vector3Double zero1 = Vector3Double.Zero;
            Vector3Double zero2 = new Vector3Double(0.0, 0.0, 0.0);
            var eq2 = zero1 == zero2;
            Assert.That(eq2, Is.True);
        }

        [Test]
        public void Vector3DoubleEqualsMethodCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v3 = new Vector3Double(1.0, 2.0, 4.0);

            using (Assert.EnterMultipleScope())
            {
                // Equals(Vector3Double)
                bool eq = v1.Equals(v2);
                Assert.That(eq, Is.True);

                eq = v1.Equals(v3);
                Assert.That(eq, Is.False);

                // Equals(object)
                eq = v1.Equals((object)v2);
                Assert.That(eq, Is.True);

                eq = v1.Equals((object)v3);
                Assert.That(eq, Is.False);

                eq = v1.Equals(null);
                Assert.That(eq, Is.False);

                eq = v1.Equals("not a vector");
                Assert.That(eq, Is.False);
            }
        }

        [Test]
        public void Vector3DoubleGetHashCodeCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v3 = new Vector3Double(1.0, 2.0, 4.0);

            // Equal vectors should have equal hash codes
            Assert.That(v1.GetHashCode(), Is.EqualTo(v2.GetHashCode()));

            // Different vectors may have different hash codes (not guaranteed, but likely)
            // We just verify it doesn't throw
            Assert.DoesNotThrow(() => _ = v3.GetHashCode());
        }

        [Test]
        public void Vector3DoubleToStringCheck()
        {
            Vector3Double v = new Vector3Double(1.0, 2.5, 3.75);
            string str = v.ToString();
            Assert.That(str, Is.EqualTo("(1, 2.5, 3.75)"));
        }

        [Test]
        public void Vector4DoubleSubtractionOperatorCheck()
        {
            Vector4Double w1 = new Vector4Double(6.0, 8.0, 10.0, 12.0);
            Vector4Double w2 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w3 = w1 - w2;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w3.X, Is.EqualTo(5.0));
                Assert.That(w3.Y, Is.EqualTo(6.0));
                Assert.That(w3.Z, Is.EqualTo(7.0));
                Assert.That(w3.W, Is.EqualTo(8.0));
            }

            w1 = new Vector4Double(0.0, 0.0, 0.0, 0.0);
            w2 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            w3 = w1 - w2;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(w3.X, Is.EqualTo(-1.0));
                Assert.That(w3.Y, Is.EqualTo(-2.0));
                Assert.That(w3.Z, Is.EqualTo(-3.0));
                Assert.That(w3.W, Is.EqualTo(-4.0));
            }
        }

        [Test]
        public void Vector4DoubleEqualityOperatorCheck()
        {
            Vector4Double w1 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w2 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w3 = new Vector4Double(1.0, 2.0, 3.0, 5.0);

            using (Assert.EnterMultipleScope())
            {
                // == operator
                bool eq = w1 == w2;
                Assert.That(eq, Is.True);

                eq = w1 == w3;
                Assert.That(eq, Is.False);

                // != operator
                eq = w1 != w3;
                Assert.That(eq, Is.True);

                eq = w1 != w2;
                Assert.That(eq, Is.False);
            }

            // Edge case: zero vectors
            Vector4Double zero1 = new Vector4Double(0.0, 0.0, 0.0, 0.0);
            Vector4Double zero2 = new Vector4Double(0.0, 0.0, 0.0, 0.0);
            var eq2 = zero1 == zero2;
            Assert.That(eq2, Is.True);
        }

        [Test]
        public void Vector4DoubleEqualsMethodCheck()
        {
            Vector4Double w1 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w2 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w3 = new Vector4Double(1.0, 2.0, 3.0, 5.0);

            using (Assert.EnterMultipleScope())
            {
                // Equals(Vector4Double)
                bool eq = w1.Equals(w2);
                Assert.That(eq, Is.True);

                eq = w1.Equals(w3);
                Assert.That(eq, Is.False);

                // Equals(object)
                eq = w1.Equals((object)w2);
                Assert.That(eq, Is.True);

                eq = w1.Equals((object)w3);
                Assert.That(eq, Is.False);

                eq = w1.Equals(null);
                Assert.That(eq, Is.False);

                eq = w1.Equals((object)w3);
                Assert.That(eq, Is.False);
            }
        }

        [Test]
        public void Vector4DoubleGetHashCodeCheck()
        {
            Vector4Double w1 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w2 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w3 = new Vector4Double(1.0, 2.0, 3.0, 5.0);

            // Equal vectors should have equal hash codes
            Assert.That(w1.GetHashCode(), Is.EqualTo(w2.GetHashCode()));

            // Different vectors may have different hash codes (not guaranteed, but likely)
            // We just verify it doesn't throw
            Assert.DoesNotThrow(() => _ = w3.GetHashCode());
        }

        [Test]
        public void Vector4DoubleToStringCheck()
        {
            Vector4Double w = new Vector4Double(1.0, 2.5, 3.75, 4.25);
            string str = w.ToString();
            Assert.That(str, Is.EqualTo("(1, 2.5, 3.75, 4.25)"));
        }

        /*
         * 
         * LinAlg tests
         * 
         */

        [Test]
        public void LinAlgTest()
        {
            double a = 0.1;
            double b = 0.1 + 1e-13;
            Assert.That(LinAlg.ApproxEqual(a, b));
            double c = 0.1 + 1e-10;
            Assert.That(LinAlg.ApproxEqual(a, c),Is.False);
            double d = 0.333333333333333333333333;
            double e = 1.0 / 3.0;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(LinAlg.ApproxEqual(d, e));
                Assert.That(LinAlg.ApproxEqual(0.0, 1e-15));

                Assert.That(LinAlg.ApproxNotEqual(a, b), Is.False);
                Assert.That(LinAlg.ApproxNotEqual(a, c));
                Assert.That(LinAlg.ApproxNotEqual(0.0, 1e-15), Is.False);
                Assert.That(LinAlg.ApproxNotEqual(1, 5));
            }

            double f = 1e-10;
            Assert.That(LinAlg.IsZero(f), Is.False);
            double g = 1e-13;
            using (Assert.EnterMultipleScope())
            {
                Assert.That(LinAlg.IsZero(g), Is.True);
                Assert.That(LinAlg.IsNotZero(f), Is.True);
                Assert.That(LinAlg.IsNotZero(g), Is.False);
            }

            // BitDecrement test
            double h = 1.0;
            double hDec = LinAlg.BitDecrement(h);
            Assert.That(hDec, Is.LessThan(h));
            double zeroDec = LinAlg.BitDecrement(0.0);
            Assert.That(zeroDec, Is.LessThan(0.0));

            // BitIncrement test
            double i = 1.0;
            double iInc = LinAlg.BitIncrement(i);
            Assert.That(iInc, Is.GreaterThan(i));
            double zeroInc = LinAlg.BitIncrement(0.0);
            Assert.That(zeroInc, Is.GreaterThan(0.0));

        }

        /*
         * Guard tests
         */
        [Test]
        public void GuardTest()
        {
            // ThrowIfNull test
            object? obj = null;
            Assert.Throws<ArgumentNullException>(() => Guard.ThrowIfNull(obj, nameof(obj)));
            obj = new object();
            Assert.DoesNotThrow(() => Guard.ThrowIfNull(obj, nameof(obj)));

            // ThrowIfNegativeOrZero test
            int value = -1;
            Assert.Throws<ArgumentOutOfRangeException>(() => Guard.ThrowIfNegativeOrZero(value, nameof(value)));
            value = 0;
            Assert.Throws<ArgumentOutOfRangeException>(() => Guard.ThrowIfNegativeOrZero(value, nameof(value)));
            value = 1;
            Assert.DoesNotThrow(() => Guard.ThrowIfNegativeOrZero(value, nameof(value)));
        }

        /*
         * BoundingBox tests
         */
        [Test]
        public void BoundingBoxBasicTest()
        {
            var min = new Vector3Double(0, 0, 0);
            var max = new Vector3Double(10, 20, 30);
            var box = new BoundingBox(min, max);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(box.Min, Is.EqualTo(min));
                Assert.That(box.Max, Is.EqualTo(max));
                Assert.That(box.IsValid, Is.True);
            }

            var expectedCenter = new Vector3Double(5, 10, 15);
            var expectedSize = new Vector3Double(10, 20, 30);
            double expectedVolume = 10 * 20 * 30;
            double expectedSurfaceArea = 2 * (10 * 20 + 20 * 30 + 30 * 10);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(box.Center, Is.EqualTo(expectedCenter));
                Assert.That(box.Size, Is.EqualTo(expectedSize));
                Assert.That(box.Volume, Is.EqualTo(expectedVolume));
                Assert.That(box.SurfaceArea, Is.EqualTo(expectedSurfaceArea));
            }

            // Invalid box
            Assert.Throws<ArgumentException>(() =>
            {
                var invalidBox = new BoundingBox(max,min);
            });
        }

        [Test]
        public void BoundingBoxFromPointsTest()
        {
            var points = new[]
            {
                new Vector3Double(1, 2, 3),
                new Vector3Double(5, 6, 7),
                new Vector3Double(-1, 0, 2),
                new Vector3Double(3, 8, 1)
            };
            // Max(5,8,7), Min(-1,0,1)
            var box = BoundingBox.FromPoints(points);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(box.Min.X, Is.EqualTo(-1));
                Assert.That(box.Min.Y, Is.EqualTo(0));
                Assert.That(box.Min.Z, Is.EqualTo(1));
                Assert.That(box.Max.X, Is.EqualTo(5));
                Assert.That(box.Max.Y, Is.EqualTo(8));
                Assert.That(box.Max.Z, Is.EqualTo(7));
            }

            // Empty test
            Assert.Throws<ArgumentException>(() => BoundingBox.FromPoints(Array.Empty<Vector3Double>()));
        }

        [Test]
        public void BoundingBoxFromControlPointsTest()
        {
            var controlPoints = new[]
            {
                new ControlPoint(0, 0, 0, 1),
                new ControlPoint(10, 0, 0, 1),
                new ControlPoint(10, 10, 0, 1),
                new ControlPoint(0, 10, 5, 1)
            };
            // Min(0,0,0), Max(10,10,5)
            var box = BoundingBox.FromControlPoints(controlPoints);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(box.Min, Is.EqualTo(new Vector3Double(0, 0, 0)));
                Assert.That(box.Max, Is.EqualTo(new Vector3Double(10, 10, 5)));
            }

            // 2D array test
            ControlPoint[][] controlPoints2D = [
                [
                
                    new ControlPoint(-5, -5, 0, 1),
                    new ControlPoint(-5, 5, 0, 1)
                ],
                [
                    new ControlPoint(5, -5, 10, 1),
                    new ControlPoint(5, 5, 10, 1)
                ]
            ];

            var box2D = BoundingBox.FromControlPoints(controlPoints2D);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(box2D.Min, Is.EqualTo(new Vector3Double(-5, -5, 0)));
                Assert.That(box2D.Max, Is.EqualTo(new Vector3Double(5, 5, 10)));
            }
        }

        [Test]
        public void BoundingBoxContainsTest()
        {
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            using (Assert.EnterMultipleScope())
            {
                // Inside points
                Assert.That(box.Contains(new Vector3Double(5, 5, 5)), Is.True);
                Assert.That(box.Contains(new Vector3Double(0, 0, 0)), Is.True);
                Assert.That(box.Contains(new Vector3Double(10, 10, 10)), Is.True);

                // Outside points
                Assert.That(box.Contains(new Vector3Double(-1, 5, 5)), Is.False);
                Assert.That(box.Contains(new Vector3Double(5, 11, 5)), Is.False);
                Assert.That(box.Contains(new Vector3Double(5, 5, 15)), Is.False);
            }
        }

        [Test]
        public void BoundingBoxIntersectsTest()
        {
            var box1 = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            // Overlapping box
            var box2 = new BoundingBox(
                new Vector3Double(5, 5, 5),
                new Vector3Double(15, 15, 15)
            );
            using (Assert.EnterMultipleScope())
            {
                Assert.That(box1.Intersects(box2), Is.True);
                Assert.That(box2.Intersects(box1), Is.True);
            }

            // Touching box
            var box3 = new BoundingBox(
                new Vector3Double(10, 0, 0),
                new Vector3Double(20, 10, 10)
            );
            Assert.That(box1.Intersects(box3), Is.True);

            // Separate box
            var box4 = new BoundingBox(
                new Vector3Double(20, 20, 20),
                new Vector3Double(30, 30, 30)
            );
            Assert.That(box1.Intersects(box4), Is.False);

            // Contained box
            var box5 = new BoundingBox(
                new Vector3Double(2, 2, 2),
                new Vector3Double(8, 8, 8)
            );
            Assert.That(box1.Intersects(box5), Is.True);
        }

        [Test]
        public void BoundingBoxUnionTest()
        {
            var box1 = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            var box2 = new BoundingBox(
                new Vector3Double(5, 5, 5),
                new Vector3Double(20, 15, 12)
            );

            var union = box1.Union(box2);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(union.Min, Is.EqualTo(new Vector3Double(0, 0, 0)));
                Assert.That(union.Max, Is.EqualTo(new Vector3Double(20, 15, 12)));
            }

            // Test commutativity
            var union2 = box2.Union(box1);
            Assert.That(union, Is.EqualTo(union2));
        }

        [Test]
        public void BoundingBoxExpandTest()
        {
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            var expanded = box.Expand(5);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(expanded.Min, Is.EqualTo(new Vector3Double(-5, -5, -5)));
                Assert.That(expanded.Max, Is.EqualTo(new Vector3Double(15, 15, 15)));
            }

            // Zero expansion
            var notExpanded = box.Expand(0);
            Assert.That(notExpanded, Is.EqualTo(box));
        }

        [Test]
        public void BoundingBoxSubdivideTest()
        {
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            var octants = box.Subdivide();

            // Should return 8 boxes
            Assert.That(octants, Has.Length.EqualTo(8));

            var center = new Vector3Double(5, 5, 5);

            // Check first octant (min corner)
            using (Assert.EnterMultipleScope())
            {
                Assert.That(octants[0].Min, Is.EqualTo(new Vector3Double(0, 0, 0)));
                Assert.That(octants[0].Max, Is.EqualTo(center));
            }

            // Check last octant (max corner)
            using (Assert.EnterMultipleScope())
            {
                Assert.That(octants[7].Min, Is.EqualTo(center));
                Assert.That(octants[7].Max, Is.EqualTo(new Vector3Double(10, 10, 10)));
            }

            // All octants should have same volume
            double expectedVolume = box.Volume / 8.0;
            foreach (var octant in octants)
            {
                Assert.That(octant.Volume, Is.EqualTo(expectedVolume).Within(1e-10));
            }
        }

        [Test]
        public void BoundingBoxClosestPointTest()
        {
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            // Point inside
            var inside = new Vector3Double(5, 5, 5);
            Assert.That(box.ClosestPoint(inside), Is.EqualTo(inside));

            // Point outside
            var outside = new Vector3Double(15, 5, 5);
            var expectedClosest = new Vector3Double(10, 5, 5);
            Assert.That(box.ClosestPoint(outside), Is.EqualTo(expectedClosest));

            // Point outside in multiple dimensions
            var outside2 = new Vector3Double(-5, 15, 5);
            var expectedClosest2 = new Vector3Double(0, 10, 5);
            Assert.That(box.ClosestPoint(outside2), Is.EqualTo(expectedClosest2));
        }

        [Test]
        public void BoundingBoxDistanceToTest()
        {
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            // Point inside (distance should be 0)
            var inside = new Vector3Double(5, 5, 5);
            Assert.That(box.DistanceTo(inside), Is.EqualTo(0));

            // Point outside on one axis
            var outside = new Vector3Double(15, 5, 5);
            Assert.That(box.DistanceTo(outside), Is.EqualTo(5));

            // Point outside on multiple axes
            var outside2 = new Vector3Double(13, 13, 10);
            double expectedDistance = Math.Sqrt(3 * 3 + 3 * 3);
            Assert.That(box.DistanceTo(outside2), Is.EqualTo(expectedDistance).Within(1e-10));
        }

        [Test]
        public void BoundingBoxEqualityTest()
        {
            var box1 = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            var box2 = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            var box3 = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(5, 5, 5)
            );

            using (Assert.EnterMultipleScope())
            {
                bool eq = box1.Equals(box2);
                Assert.That(eq, Is.True);

                eq = box1 == box2;
                Assert.That(eq, Is.True);

                eq = box1.Equals(box3);
                Assert.That(eq, Is.False);

                eq = box1 != box3;
                Assert.That(eq, Is.True);
            }

            // GetHashCode test
            Assert.That(box1.GetHashCode(), Is.EqualTo(box2.GetHashCode()));
        }

        [Test]
        public void BoundingBoxToStringTest()
        {
            var box = new BoundingBox(
                new Vector3Double(0, 0, 0),
                new Vector3Double(10, 10, 10)
            );

            var str = box.ToString();
            Assert.That(str, Does.Contain("BoundingBox"));
            Assert.That(str, Does.Contain("Min"));
            Assert.That(str, Does.Contain("Max"));
        }

        /*
         * Ray tests
         */
        [Test]
        public void RayBasicTest()
        {
            // Constructor test with normalization
            var origin = new Vector3Double(1, 2, 3);
            var direction = new Vector3Double(3, 0, 4); // magnitude = 5
            var ray = new Ray(origin, direction);

            using (Assert.EnterMultipleScope())
            {
                Assert.That(ray.Origin, Is.EqualTo(origin));
                Assert.That(ray.IsNormalized, Is.True);
                Assert.That(ray.Direction.magnitude, Is.EqualTo(1.0).Within(1e-10));
            }

            // Constructor without normalization
            var rayUnnormalized = new Ray(origin, direction, normalize: false);
            using (Assert.EnterMultipleScope())
            {
                Assert.That(rayUnnormalized.IsNormalized, Is.False);
                Assert.That(rayUnnormalized.Direction, Is.EqualTo(direction));
            }

            // Zero vector should throw
            Assert.Throws<ArgumentException>(() => new Ray(origin, Vector3Double.Zero));
        }

        [Test]
        public void RayPointAtTest()
        {
            var origin = new Vector3Double(1, 2, 3);
            var direction = new Vector3Double(1, 0, 0); //normalized
            var ray = new Ray(origin, direction);

            // t = 0 should return origin
            Assert.That(ray.PointAt(0), Is.EqualTo(origin));// (1,2,3) * (0,0,0)

            // t = 5 should return origin + 5 * direction
            var expected = new Vector3Double(6, 2, 3);
            Assert.That(ray.PointAt(5), Is.EqualTo(expected));//(1,2,3) + 5*(1,0,0) = (6,2,3)

            // Negative t (mathematically valid, but represents opposite direction)
            var negative = ray.PointAt(-2);
            Assert.That(negative, Is.EqualTo(new Vector3Double(-1, 2, 3)));// (1,2,3) + -2*(1,0,0) = (-1,2,3)
        }


        [Test]
        public void RayNormalizedTest()
        {
            var origin = new Vector3Double(1, 2, 3);
            var direction = new Vector3Double(3, 0, 4); // magnitude = 5
            
            // Create unnormalized ray
            var ray = new Ray(origin, direction, normalize: false);
            Assert.That(ray.IsNormalized, Is.False);

            // Get normalized version
            var normalized = ray.Normalized();
            using (Assert.EnterMultipleScope())
            {
                Assert.That(normalized.IsNormalized, Is.True);
                Assert.That(normalized.Direction.magnitude, Is.EqualTo(1.0).Within(1e-10));
                Assert.That(normalized.Origin, Is.EqualTo(origin));
            }

            // Normalizing already normalized ray should return same ray
            var alreadyNormalized = new Ray(origin, direction, normalize: true);
            var normalized2 = alreadyNormalized.Normalized();
            Assert.That(normalized2, Is.EqualTo(alreadyNormalized));
        }

        [Test]
        public void RayTranslateTest()
        {
            var origin = new Vector3Double(1, 2, 3);
            var direction = new Vector3Double(1, 0, 0);
            var ray = new Ray(origin, direction);

            var offset = new Vector3Double(5, 10, 15);
            var translated = ray.Translate(offset);// (1,2,3) + (5,10,15) = (6,12,18)

            using (Assert.EnterMultipleScope())
            {
                Assert.That(translated.Origin, Is.EqualTo(new Vector3Double(6, 12, 18)));
                Assert.That(translated.Direction, Is.EqualTo(direction));
            }

            // Zero translation
            var notTranslated = ray.Translate(Vector3Double.Zero);
            Assert.That(notTranslated.Origin, Is.EqualTo(origin));
        }

        [Test]
        public void RayEqualityTest()
        {
            var origin1 = new Vector3Double(1, 2, 3);
            var direction1 = new Vector3Double(1, 0, 0);
            var ray1 = new Ray(origin1, direction1);
            var ray2 = new Ray(origin1, direction1);

            var origin2 = new Vector3Double(2, 3, 4);
            var ray3 = new Ray(origin2, direction1);

            using (Assert.EnterMultipleScope())
            {
                bool eq = ray1 == ray2;
                Assert.That(eq, Is.True);

                eq = ray1.Equals(ray2);
                Assert.That(eq, Is.True);

                eq = ray1 != ray3;
                Assert.That(eq, Is.True);
                
                eq = ray1.Equals(ray3);
                Assert.That(eq, Is.False);
            }

            // GetHashCode test
            Assert.That(ray1.GetHashCode(), Is.EqualTo(ray2.GetHashCode()));
        }

        [Test]
        public void RayToStringTest()
        {
            var ray = new Ray(
                new Vector3Double(1, 2, 3),
                new Vector3Double(1, 0, 0)
            );

            var str = ray.ToString();
            Assert.That(str, Does.Contain("Ray"));
            Assert.That(str, Does.Contain("Origin"));
            Assert.That(str, Does.Contain("Direction"));
        }

        [Test]
        public void RayNormalizationAccuracyTest()
        {
            // Test various direction vectors for normalization accuracy
            var testVectors = new[]
            {
                new Vector3Double(1, 1, 1),
                new Vector3Double(3, 4, 0),
                new Vector3Double(1, 2, 2),
                new Vector3Double(0.1, 0.2, 0.3),
                new Vector3Double(100, 200, 300)
            };

            foreach (var direction in testVectors)
            {
                var ray = new Ray(Vector3Double.Zero, direction, normalize: true);
                Assert.That(ray.Direction.magnitude, Is.EqualTo(1.0).Within(1e-10),
                    $"Direction {direction} failed normalization");
            }
        }
    }
}
