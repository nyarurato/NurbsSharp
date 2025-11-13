using System;
using System.Numerics;
#pragma warning disable CS1591


namespace NurbsSharp.Core
{

    /// <summary>
    /// (en) 3D vector with double precision
    /// (ja) 倍精度浮動小数点数を使用した3次元ベクトル
    /// </summary>
    public class Vector3Double
    {
        private readonly double[] _values = new double[3];

        public double X
        {
            get => _values[0];
            set => _values[0] = value;
        }
        public double Y
        {
            get => _values[1];
            set => _values[1] = value;
        }
        public double Z
        {
            get => _values[2];
            set => _values[2] = value;
        }

        /// <summary>
        /// L2 norm
        /// </summary>
        public double magnitude
        {
            get => Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        /// <summary>
        /// return normalized vector (1,2,2) => (1/sqrt(5),2/sqrt(5),2/sqrt(5))
        /// </summary>
        public Vector3Double normalized
        {
            get{ 
                double mag = magnitude;
                if (mag == 0)
                    throw new InvalidOperationException("Cannot normalize a zero-length vector.");
                return new Vector3Double(X / mag, Y / mag, Z / mag);
            }
        }

        public static Vector3Double Zero => new Vector3Double(0.0, 0.0, 0.0);

        /// <summary>
        /// Constructor
        /// </summary>
        public Vector3Double()
        {
            _values[0] = 0.0;
            _values[1] = 0.0;
            _values[2] = 0.0;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        public Vector3Double(double x, double y, double z)
        {
            _values[0] = x;
            _values[1] = y;
            _values[2] = z;
        }
        /// <summary>
        /// (en) Calculate the distance to another vector
        /// (ja) 他のベクトルまでの距離を計算する
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public double DistanceTo(Vector3Double other)
        {
            double dx = X - other.X;
            double dy = Y - other.Y;
            double dz = Z - other.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// (en) Convert to System.Numerics.Vector3
        /// (ja) System.Numerics.Vector3 に変換する
        /// </summary>
        /// <returns></returns>
        public Vector3 ToVector3()
        {
            return new Vector3((float)X, (float)Y, (float)Z);
        }

        public static Vector3Double operator *(Vector3Double v, double scalar)
        {
            return new Vector3Double(v.X * scalar, v.Y * scalar, v.Z * scalar);
        }

        public static Vector3Double operator *(double scalar, Vector3Double v)
        {
            return v * scalar;
        }

        public static Vector3Double operator +(Vector3Double a, Vector3Double b)
        {
            return new Vector3Double(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        public static Vector3Double operator -(Vector3Double a, Vector3Double b)
        {
            return new Vector3Double(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        public static Vector3Double operator -(Vector3Double v)
        {
            return new Vector3Double(-v.X, -v.Y, -v.Z);
        }

        public static Vector3Double operator /(Vector3Double v, double scalar)
        {
            if (scalar == 0)
                throw new DivideByZeroException("Cannot divide by zero.");
            return new Vector3Double(v.X / scalar, v.Y / scalar, v.Z / scalar);
        }

        public static explicit operator Vector3(Vector3Double v)
        {
            return v.ToVector3();
        }

        /// <summary>
        /// dot product
        /// (a1,a2,a3).(b1,b2,b3) = a1*b1 + a2*b2 + a3*b3
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static double Dot(Vector3Double a, Vector3Double b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// <summary>
        /// cross product
        /// (a1,a2,a3)x(b1,b2,b3) = (a2*b3 - a3*b2, a3*b1 - a1*b3, a1*b2 - a2*b1)
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static Vector3Double Cross(Vector3Double a, Vector3Double b)
        {
            return new Vector3Double(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X
            );
        }

        /// <summary>
        /// return string representation
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }
    }

    /// <summary>
    /// (en) 4D vector with double precision
    /// (ja) 倍精度浮動小数点数を使用した4次元ベクトル
    /// </summary>
    public class Vector4Double
    {
        private readonly double[] _values = new double[4];

        public double X
        {
            get => _values[0];
            set => _values[0] = value;
        }
        public double Y
        {
            get => _values[1];
            set => _values[1] = value;
        }
        public double Z
        {
            get => _values[2];
            set => _values[2] = value;
        }
        public double W
        {
            get => _values[3];
            set => _values[3] = value;
        }

        /// <summary>
        /// Constructor
        /// </summary>
        public Vector4Double()
        {
            _values[0] = 0.0;
            _values[1] = 0.0;
            _values[2] = 0.0;
            _values[3] = 0.0;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="w"></param>
        public Vector4Double(double x, double y, double z, double w)
        {
            _values[0] = x;
            _values[1] = y;
            _values[2] = z;
            _values[3] = w;
        }

        public static Vector4Double operator *(Vector4Double v, double scalar)
        {
            return new Vector4Double(v.X * scalar, v.Y * scalar, v.Z * scalar, v.W * scalar);
        }

        public static Vector4Double operator *(double scalar, Vector4Double v)
        {
            return v * scalar;
        }

        public static Vector4Double operator +(Vector4Double a, Vector4Double b)
        {
            return new Vector4Double(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);
        }

        public static Vector4Double operator -(Vector4Double a, Vector4Double b)
        {
            return new Vector4Double(a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W);
        }
        public static Vector4Double operator -(Vector4Double v)
        {
            return new Vector4Double(-v.X, -v.Y, -v.Z, -v.W);
        }
        public static Vector4Double operator /(Vector4Double v, double scalar)
        {
            if (scalar == 0)
                throw new DivideByZeroException("Cannot divide by zero.");
            return new Vector4Double(v.X / scalar, v.Y / scalar, v.Z / scalar, v.W / scalar);
        }

        /// <summary>
        /// return string representation
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"({X}, {Y}, {Z}, {W})";
        }
    }

}
#pragma warning restore CS1591
