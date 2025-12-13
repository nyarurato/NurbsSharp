using NUnit.Framework;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using System;

namespace NurbsSharp.UnitTests.Operation
{
    [TestFixture]
    public class TransformOperatorTests
    {
        private NurbsCurve CreateTestCurve()
        {
            // Create a simple 2nd degree curve
            int degree = 2;
            var knots = new KnotVector(new double[] { 0, 0, 0, 1, 1, 1 }, degree);
            var controlPoints = new ControlPoint[]
            {
                new ControlPoint(0, 0, 0),
                new ControlPoint(1, 1, 0),
                new ControlPoint(2, 0, 0)
            };
            return new NurbsCurve(degree, knots, controlPoints);
        }

        private NurbsSurface CreateTestSurface()
        {
            // Create a simple bilinear surface
            int degreeU = 1;
            int degreeV = 1;
            var knotsU = new KnotVector(new double[] { 0, 0, 1, 1 }, degreeU);
            var knotsV = new KnotVector(new double[] { 0, 0, 1, 1 }, degreeV);
            var controlPoints = new ControlPoint[][]
            {
                new ControlPoint[] { new ControlPoint(0, 0, 0), new ControlPoint(0, 1, 0) },
                new ControlPoint[] { new ControlPoint(1, 0, 0), new ControlPoint(1, 1, 0) }
            };
            return new NurbsSurface(degreeU, degreeV, knotsU, knotsV, controlPoints);
        }

        #region Matrix4x4 Tests

        [Test]
        public void Matrix4x4_Identity_Test()
        {
            var identity = Matrix4x4.Identity;
            var point = new Vector3Double(1, 2, 3);
            var result = identity.TransformPoint(point);

            Assert.That(result.X, Is.EqualTo(1).Within(1e-10));
            Assert.That(result.Y, Is.EqualTo(2).Within(1e-10));
            Assert.That(result.Z, Is.EqualTo(3).Within(1e-10));
        }

        [Test]
        public void Matrix4x4_Translation_Test()
        {
            var translation = Matrix4x4.CreateTranslation(10, 20, 30);
            var point = new Vector3Double(1, 2, 3);
            var result = translation.TransformPoint(point);

            Assert.That(result.X, Is.EqualTo(11).Within(1e-10));
            Assert.That(result.Y, Is.EqualTo(22).Within(1e-10));
            Assert.That(result.Z, Is.EqualTo(33).Within(1e-10));
        }

        [Test]
        public void Matrix4x4_Scale_Test()
        {
            var scale = Matrix4x4.CreateScale(2, 3, 4);
            var point = new Vector3Double(1, 2, 3);
            var result = scale.TransformPoint(point);

            Assert.That(result.X, Is.EqualTo(2).Within(1e-10));
            Assert.That(result.Y, Is.EqualTo(6).Within(1e-10));
            Assert.That(result.Z, Is.EqualTo(12).Within(1e-10));
        }

        [Test]
        public void Matrix4x4_RotationZ_90Degrees_Test()
        {
            var rotation = Matrix4x4.CreateRotationZ(Math.PI / 2); // 90 degrees
            var point = new Vector3Double(1, 0, 0);
            var result = rotation.TransformPoint(point);

            Assert.That(result.X, Is.EqualTo(0).Within(1e-10));
            Assert.That(result.Y, Is.EqualTo(1).Within(1e-10));
            Assert.That(result.Z, Is.EqualTo(0).Within(1e-10));
        }

        [Test]
        public void Matrix4x4_Multiplication_Test()
        {
            var translation = Matrix4x4.CreateTranslation(1, 2, 3);
            var scale = Matrix4x4.CreateScale(2);
            var combined = translation * scale; // Translation then scale
            var point = new Vector3Double(1, 1, 1);
            var result = combined.TransformPoint(point);

            // Scale first, then translate: (2,2,2) + (1,2,3) = (3,4,5)
            Assert.That(result.X, Is.EqualTo(3).Within(1e-10));
            Assert.That(result.Y, Is.EqualTo(4).Within(1e-10));
            Assert.That(result.Z, Is.EqualTo(5).Within(1e-10));
        }

        #endregion

        #region Curve Translation Tests

        [Test]
        public void Curve_Translate_NonDestructive_Test()
        {
            var curve = CreateTestCurve();
            var originalPos = curve.ControlPoints[0].Position;
            var delta = new Vector3Double(10, 20, 30);

            var newCurve = TransformOperator.Translate(curve, delta);

            // Original should be unchanged
            Assert.That(curve.ControlPoints[0].Position.X, Is.EqualTo(originalPos.X).Within(1e-10));
            Assert.That(curve.ControlPoints[0].Position.Y, Is.EqualTo(originalPos.Y).Within(1e-10));
            Assert.That(curve.ControlPoints[0].Position.Z, Is.EqualTo(originalPos.Z).Within(1e-10));

            // New curve should be translated
            Assert.That(newCurve.ControlPoints[0].Position.X, Is.EqualTo(originalPos.X + 10).Within(1e-10));
            Assert.That(newCurve.ControlPoints[0].Position.Y, Is.EqualTo(originalPos.Y + 20).Within(1e-10));
            Assert.That(newCurve.ControlPoints[0].Position.Z, Is.EqualTo(originalPos.Z + 30).Within(1e-10));
        }

        [Test]
        public void Curve_Translate_InPlace_Test()
        {
            var curve = CreateTestCurve();
            var originalPos = curve.ControlPoints[0].Position;
            var delta = new Vector3Double(10, 20, 30);

            curve.Translate(delta);

            Assert.That(curve.ControlPoints[0].Position.X, Is.EqualTo(originalPos.X + 10).Within(1e-10));
            Assert.That(curve.ControlPoints[0].Position.Y, Is.EqualTo(originalPos.Y + 20).Within(1e-10));
            Assert.That(curve.ControlPoints[0].Position.Z, Is.EqualTo(originalPos.Z + 30).Within(1e-10));
        }

        #endregion

        #region Curve Rotation Tests

        [Test]
        public void Curve_Rotate_Z_90Degrees_Test()
        {
            var curve = CreateTestCurve();
            var axis = new Vector3Double(0, 0, 1);
            var angle = Math.PI / 2; // 90 degrees

            var newCurve = TransformOperator.Rotate(curve, axis, angle);

            // Point (1,1,0) should rotate to approximately (-1,1,0)
            Assert.That(newCurve.ControlPoints[1].Position.X, Is.EqualTo(-1).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.Y, Is.EqualTo(1).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.Z, Is.EqualTo(0).Within(1e-10));
        }

        [Test]
        public void Curve_Rotate_WithCenter_Test()
        {
            var curve = CreateTestCurve();
            var axis = new Vector3Double(0, 0, 1);
            var angle = Math.PI; // 180 degrees
            var center = new Vector3Double(1, 0, 0);

            var newCurve = TransformOperator.Rotate(curve, axis, angle, center);

            // Point (0,0,0) rotated 180° around (1,0,0) should be (2,0,0)
            Assert.That(newCurve.ControlPoints[0].Position.X, Is.EqualTo(2).Within(1e-10));
            Assert.That(newCurve.ControlPoints[0].Position.Y, Is.EqualTo(0).Within(1e-10));
            Assert.That(newCurve.ControlPoints[0].Position.Z, Is.EqualTo(0).Within(1e-10));
        }

        [Test]
        public void Curve_Rotate_InPlace_Test()
        {
            var curve = CreateTestCurve();
            var axis = new Vector3Double(0, 0, 1);
            var angle = Math.PI / 2;

            curve.Rotate(axis, angle);

            Assert.That(curve.ControlPoints[1].Position.X, Is.EqualTo(-1).Within(1e-10));
            Assert.That(curve.ControlPoints[1].Position.Y, Is.EqualTo(1).Within(1e-10));
        }

        #endregion

        #region Curve Scale Tests

        [Test]
        public void Curve_Scale_Uniform_Test()
        {
            var curve = CreateTestCurve();
            var scale = 2.0;

            var newCurve = TransformOperator.Scale(curve, scale);

            Assert.That(newCurve.ControlPoints[0].Position.X, Is.EqualTo(0).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.X, Is.EqualTo(2).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.Y, Is.EqualTo(2).Within(1e-10));
            Assert.That(newCurve.ControlPoints[2].Position.X, Is.EqualTo(4).Within(1e-10));
        }

        [Test]
        public void Curve_Scale_NonUniform_Test()
        {
            var curve = CreateTestCurve();

            var newCurve = TransformOperator.Scale(curve, 2, 3, 1);

            Assert.That(newCurve.ControlPoints[1].Position.X, Is.EqualTo(2).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.Y, Is.EqualTo(3).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.Z, Is.EqualTo(0).Within(1e-10));
        }

        [Test]
        public void Curve_Scale_WithCenter_Test()
        {
            var curve = CreateTestCurve();
            var center = new Vector3Double(1, 0, 0);
            var scale = 2.0;

            var newCurve = TransformOperator.Scale(curve, scale, center);

            // Point (0,0,0) scaled 2x around (1,0,0): (0-1)*2 + 1 = -1
            Assert.That(newCurve.ControlPoints[0].Position.X, Is.EqualTo(-1).Within(1e-10));
            // Point (1,1,0) scaled 2x around (1,0,0): ((1,1)-(1,0))*2 + (1,0) = (1,2)
            Assert.That(newCurve.ControlPoints[1].Position.X, Is.EqualTo(1).Within(1e-10));
            Assert.That(newCurve.ControlPoints[1].Position.Y, Is.EqualTo(2).Within(1e-10));
        }

        [Test]
        public void Curve_Scale_InPlace_Test()
        {
            var curve = CreateTestCurve();
            var scale = 2.0;

            curve.Scale(scale);

            Assert.That(curve.ControlPoints[1].Position.X, Is.EqualTo(2).Within(1e-10));
            Assert.That(curve.ControlPoints[1].Position.Y, Is.EqualTo(2).Within(1e-10));
        }

        #endregion

        #region Surface Tests

        [Test]
        public void Surface_Translate_Test()
        {
            var surface = CreateTestSurface();
            var delta = new Vector3Double(5, 10, 15);

            var newSurface = TransformOperator.Translate(surface, delta);

            Assert.That(newSurface.ControlPoints[0][0].Position.X, Is.EqualTo(5).Within(1e-10));
            Assert.That(newSurface.ControlPoints[0][0].Position.Y, Is.EqualTo(10).Within(1e-10));
            Assert.That(newSurface.ControlPoints[0][0].Position.Z, Is.EqualTo(15).Within(1e-10));
        }

        [Test]
        public void Surface_Rotate_Test()
        {
            var surface = CreateTestSurface();
            var axis = new Vector3Double(0, 0, 1);
            var angle = Math.PI / 2;

            var newSurface = TransformOperator.Rotate(surface, axis, angle);

            // Point (1,0,0) rotated 90° around Z becomes (0,1,0)
            Assert.That(newSurface.ControlPoints[1][0].Position.X, Is.EqualTo(0).Within(1e-10));
            Assert.That(newSurface.ControlPoints[1][0].Position.Y, Is.EqualTo(1).Within(1e-10));
        }

        [Test]
        public void Surface_Scale_Test()
        {
            var surface = CreateTestSurface();

            var newSurface = TransformOperator.Scale(surface, 2.0);

            Assert.That(newSurface.ControlPoints[1][1].Position.X, Is.EqualTo(2).Within(1e-10));
            Assert.That(newSurface.ControlPoints[1][1].Position.Y, Is.EqualTo(2).Within(1e-10));
        }

        [Test]
        public void Surface_Transform_InPlace_Test()
        {
            var surface = CreateTestSurface();
            var matrix = Matrix4x4.CreateTranslation(10, 20, 30);

            surface.Transform(matrix);

            Assert.That(surface.ControlPoints[0][0].Position.X, Is.EqualTo(10).Within(1e-10));
            Assert.That(surface.ControlPoints[0][0].Position.Y, Is.EqualTo(20).Within(1e-10));
            Assert.That(surface.ControlPoints[0][0].Position.Z, Is.EqualTo(30).Within(1e-10));
        }

        #endregion

        #region Complex Transformation Tests

        [Test]
        public void Curve_ComplexTransformation_Test()
        {
            var curve = CreateTestCurve();
            
            // Translate, then rotate, then scale
            var translation = Matrix4x4.CreateTranslation(1, 0, 0);
            var rotation = Matrix4x4.CreateRotationZ(Math.PI / 2);
            var scale = Matrix4x4.CreateScale(2);
            
            var combined = scale * rotation * translation;
            var newCurve = TransformOperator.Transform(curve, combined);

            // Verify curve is not null and has correct structure
            Assert.That(newCurve, Is.Not.Null);
            Assert.That(newCurve.ControlPoints.Length, Is.EqualTo(3));
        }

        [Test]
        public void Curve_PreserveWeights_AfterTransform_Test()
        {
            var curve = CreateTestCurve();
            curve.ControlPoints[1].Weight = 2.0; // Set non-uniform weight

            var newCurve = TransformOperator.Scale(curve, 2.0);

            Assert.That(newCurve.ControlPoints[1].Weight, Is.EqualTo(2.0).Within(1e-10));
        }

        #endregion
    }
}
