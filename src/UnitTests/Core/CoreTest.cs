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

            // == operator
            Assert.That(v1 == v2, Is.True);
            Assert.That(v1 == v3, Is.False);

            // != operator
            Assert.That(v1 != v3, Is.True);
            Assert.That(v1 != v2, Is.False);

            // Edge case: zero vectors
            Vector3Double zero1 = Vector3Double.Zero;
            Vector3Double zero2 = new Vector3Double(0.0, 0.0, 0.0);
            Assert.That(zero1 == zero2, Is.True);
        }

        [Test]
        public void Vector3DoubleEqualsMethodCheck()
        {
            Vector3Double v1 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v2 = new Vector3Double(1.0, 2.0, 3.0);
            Vector3Double v3 = new Vector3Double(1.0, 2.0, 4.0);

            // Equals(Vector3Double)
            Assert.That(v1.Equals(v2), Is.True);
            Assert.That(v1.Equals(v3), Is.False);

            // Equals(object)
            Assert.That(v1.Equals((object)v2), Is.True);
            Assert.That(v1.Equals((object)v3), Is.False);
            Assert.That(v1.Equals(null), Is.False);
            Assert.That(v1.Equals("not a vector"), Is.False);
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

            // == operator
            Assert.That(w1 == w2, Is.True);
            Assert.That(w1 == w3, Is.False);

            // != operator
            Assert.That(w1 != w3, Is.True);
            Assert.That(w1 != w2, Is.False);

            // Edge case: zero vectors
            Vector4Double zero1 = new Vector4Double(0.0, 0.0, 0.0, 0.0);
            Vector4Double zero2 = new Vector4Double(0.0, 0.0, 0.0, 0.0);
            Assert.That(zero1 == zero2, Is.True);
        }

        [Test]
        public void Vector4DoubleEqualsMethodCheck()
        {
            Vector4Double w1 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w2 = new Vector4Double(1.0, 2.0, 3.0, 4.0);
            Vector4Double w3 = new Vector4Double(1.0, 2.0, 3.0, 5.0);

            // Equals(Vector4Double)
            Assert.That(w1.Equals(w2), Is.True);
            Assert.That(w1.Equals(w3), Is.False);

            // Equals(object)
            Assert.That(w1.Equals((object)w2), Is.True);
            Assert.That(w1.Equals((object)w3), Is.False);
            Assert.That(w1.Equals(null), Is.False);
            Assert.That(w1.Equals("not a vector"), Is.False);
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
    }
}
