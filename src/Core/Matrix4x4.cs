using System;

namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) 4x4 transformation matrix for affine transformations
    /// (ja) アフィン変換用の4x4変換行列
    /// </summary>
    public readonly struct Matrix4x4 : IEquatable<Matrix4x4>
    {
        // Matrix elements (row-major order)
        public readonly double M11, M12, M13, M14;
        public readonly double M21, M22, M23, M24;
        public readonly double M31, M32, M33, M34;
        public readonly double M41, M42, M43, M44;

        /// <summary>
        /// Constructor
        /// </summary>
        public Matrix4x4(
            double m11, double m12, double m13, double m14,
            double m21, double m22, double m23, double m24,
            double m31, double m32, double m33, double m34,
            double m41, double m42, double m43, double m44)
        {
            M11 = m11; M12 = m12; M13 = m13; M14 = m14;
            M21 = m21; M22 = m22; M23 = m23; M24 = m24;
            M31 = m31; M32 = m32; M33 = m33; M34 = m34;
            M41 = m41; M42 = m42; M43 = m43; M44 = m44;
        }

        /// <summary>
        /// Identity matrix
        /// </summary>
        public static Matrix4x4 Identity => new Matrix4x4(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);

        /// <summary>
        /// (en) Create translation matrix
        /// (ja) 平行移動行列を作成
        /// </summary>
        public static Matrix4x4 CreateTranslation(Vector3Double translation)
        {
            return CreateTranslation(translation.X, translation.Y, translation.Z);
        }

        /// <summary>
        /// (en) Create translation matrix
        /// (ja) 平行移動行列を作成
        /// </summary>
        public static Matrix4x4 CreateTranslation(double x, double y, double z)
        {
            return new Matrix4x4(
                1, 0, 0, x,
                0, 1, 0, y,
                0, 0, 1, z,
                0, 0, 0, 1);
        }

        /// <summary>
        /// (en) Create scale matrix
        /// (ja) スケール行列を作成
        /// </summary>
        public static Matrix4x4 CreateScale(double scale)
        {
            return CreateScale(scale, scale, scale);
        }

        /// <summary>
        /// (en) Create scale matrix
        /// (ja) スケール行列を作成
        /// </summary>
        public static Matrix4x4 CreateScale(double sx, double sy, double sz)
        {
            return new Matrix4x4(
                sx, 0, 0, 0,
                0, sy, 0, 0,
                0, 0, sz, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// (en) Create rotation matrix around X axis
        /// (ja) X軸周りの回転行列を作成
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        public static Matrix4x4 CreateRotationX(double angle)
        {
            double c = Math.Cos(angle);
            double s = Math.Sin(angle);
            return new Matrix4x4(
                1, 0, 0, 0,
                0, c, -s, 0,
                0, s, c, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// (en) Create rotation matrix around Y axis
        /// (ja) Y軸周りの回転行列を作成
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        public static Matrix4x4 CreateRotationY(double angle)
        {
            double c = Math.Cos(angle);
            double s = Math.Sin(angle);
            return new Matrix4x4(
                c, 0, s, 0,
                0, 1, 0, 0,
                -s, 0, c, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// (en) Create rotation matrix around Z axis
        /// (ja) Z軸周りの回転行列を作成
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        public static Matrix4x4 CreateRotationZ(double angle)
        {
            double c = Math.Cos(angle);
            double s = Math.Sin(angle);
            return new Matrix4x4(
                c, -s, 0, 0,
                s, c, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// (en) Create rotation matrix around arbitrary axis
        /// (ja) 任意の軸周りの回転行列を作成
        /// </summary>
        /// <param name="axis">Rotation axis (will be normalized)</param>
        /// <param name="angle">Angle in radians</param>
        public static Matrix4x4 CreateRotation(Vector3Double axis, double angle)
        {
            Vector3Double n = axis.normalized;
            double c = Math.Cos(angle);
            double s = Math.Sin(angle);
            double t = 1 - c;

            return new Matrix4x4(
                t * n.X * n.X + c, t * n.X * n.Y - s * n.Z, t * n.X * n.Z + s * n.Y, 0,
                t * n.X * n.Y + s * n.Z, t * n.Y * n.Y + c, t * n.Y * n.Z - s * n.X, 0,
                t * n.X * n.Z - s * n.Y, t * n.Y * n.Z + s * n.X, t * n.Z * n.Z + c, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// (en) Create rotation matrix around arbitrary axis with center point
        /// (ja) 任意の軸周りの回転行列を作成（中心点指定）
        /// </summary>
        /// <param name="axis">Rotation axis (will be normalized)</param>
        /// <param name="angle">Angle in radians</param>
        /// <param name="center">Center of rotation</param>
        public static Matrix4x4 CreateRotation(Vector3Double axis, double angle, Vector3Double center)
        {
            Matrix4x4 translate1 = CreateTranslation(-center.X, -center.Y, -center.Z);
            Matrix4x4 rotate = CreateRotation(axis, angle);
            Matrix4x4 translate2 = CreateTranslation(center.X, center.Y, center.Z);
            return translate2 * rotate * translate1;
        }

        /// <summary>
        /// (en) Create scale matrix with center point
        /// (ja) スケール行列を作成（中心点指定）
        /// </summary>
        public static Matrix4x4 CreateScale(double sx, double sy, double sz, Vector3Double center)
        {
            Matrix4x4 translate1 = CreateTranslation(-center.X, -center.Y, -center.Z);
            Matrix4x4 scale = CreateScale(sx, sy, sz);
            Matrix4x4 translate2 = CreateTranslation(center.X, center.Y, center.Z);
            return translate2 * scale * translate1;
        }

        /// <summary>
        /// (en) Matrix multiplication
        /// (ja) 行列の乗算
        /// </summary>
        public static Matrix4x4 operator *(Matrix4x4 a, Matrix4x4 b)
        {
            return new Matrix4x4(
                a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31 + a.M14 * b.M41,
                a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32 + a.M14 * b.M42,
                a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33 + a.M14 * b.M43,
                a.M11 * b.M14 + a.M12 * b.M24 + a.M13 * b.M34 + a.M14 * b.M44,

                a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31 + a.M24 * b.M41,
                a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32 + a.M24 * b.M42,
                a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33 + a.M24 * b.M43,
                a.M21 * b.M14 + a.M22 * b.M24 + a.M23 * b.M34 + a.M24 * b.M44,

                a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31 + a.M34 * b.M41,
                a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32 + a.M34 * b.M42,
                a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33 + a.M34 * b.M43,
                a.M31 * b.M14 + a.M32 * b.M24 + a.M33 * b.M34 + a.M34 * b.M44,

                a.M41 * b.M11 + a.M42 * b.M21 + a.M43 * b.M31 + a.M44 * b.M41,
                a.M41 * b.M12 + a.M42 * b.M22 + a.M43 * b.M32 + a.M44 * b.M42,
                a.M41 * b.M13 + a.M42 * b.M23 + a.M43 * b.M33 + a.M44 * b.M43,
                a.M41 * b.M14 + a.M42 * b.M24 + a.M43 * b.M34 + a.M44 * b.M44);
        }

        /// <summary>
        /// (en) Transform a 3D point (applies translation)
        /// (ja) 3D点を変換（平行移動を適用）
        /// </summary>
        public Vector3Double TransformPoint(Vector3Double point)
        {
            double x = point.X * M11 + point.Y * M12 + point.Z * M13 + M14;
            double y = point.X * M21 + point.Y * M22 + point.Z * M23 + M24;
            double z = point.X * M31 + point.Y * M32 + point.Z * M33 + M34;
            double w = point.X * M41 + point.Y * M42 + point.Z * M43 + M44;

            if (Math.Abs(w - 1.0) > 1e-10)
            {
                return new Vector3Double(x / w, y / w, z / w);
            }
            return new Vector3Double(x, y, z);
        }

        /// <summary>
        /// (en) Transform a 3D vector (ignores translation)
        /// (ja) 3Dベクトルを変換（平行移動を無視）
        /// </summary>
        public Vector3Double TransformVector(Vector3Double vector)
        {
            double x = vector.X * M11 + vector.Y * M12 + vector.Z * M13;
            double y = vector.X * M21 + vector.Y * M22 + vector.Z * M23;
            double z = vector.X * M31 + vector.Y * M32 + vector.Z * M33;
            return new Vector3Double(x, y, z);
        }

        public bool Equals(Matrix4x4 other)
        {
            return M11 == other.M11 && M12 == other.M12 && M13 == other.M13 && M14 == other.M14 &&
                   M21 == other.M21 && M22 == other.M22 && M23 == other.M23 && M24 == other.M24 &&
                   M31 == other.M31 && M32 == other.M32 && M33 == other.M33 && M34 == other.M34 &&
                   M41 == other.M41 && M42 == other.M42 && M43 == other.M43 && M44 == other.M44;
        }

        public override bool Equals(object? obj)
        {
            return obj is Matrix4x4 other && Equals(other);
        }

        public override int GetHashCode()
        {
            var hash = new HashCode();
            hash.Add(M11); hash.Add(M12); hash.Add(M13); hash.Add(M14);
            hash.Add(M21); hash.Add(M22); hash.Add(M23); hash.Add(M24);
            hash.Add(M31); hash.Add(M32); hash.Add(M33); hash.Add(M34);
            hash.Add(M41); hash.Add(M42); hash.Add(M43); hash.Add(M44);
            return hash.ToHashCode();
        }

        public static bool operator ==(Matrix4x4 left, Matrix4x4 right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Matrix4x4 left, Matrix4x4 right)
        {
            return !left.Equals(right);
        }

        public override string ToString()
        {
            return $"Matrix4x4(\n" +
                   $"  [{M11:F3}, {M12:F3}, {M13:F3}, {M14:F3}]\n" +
                   $"  [{M21:F3}, {M22:F3}, {M23:F3}, {M24:F3}]\n" +
                   $"  [{M31:F3}, {M32:F3}, {M33:F3}, {M34:F3}]\n" +
                   $"  [{M41:F3}, {M42:F3}, {M43:F3}, {M44:F3}])";
        }
    }
}
