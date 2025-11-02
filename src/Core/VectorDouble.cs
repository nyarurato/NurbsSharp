using System;
using System.Numerics;

namespace NurbsSharp.Core
{

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

        public Vector3Double()
        {
            _values[0] = 0.0;
            _values[1] = 0.0;
            _values[2] = 0.0;
        }
        public Vector3Double(double x, double y, double z)
        {
            _values[0] = x;
            _values[1] = y;
            _values[2] = z;
        }

        public double DistanceTo(Vector3Double other)
        {
            double dx = X - other.X;
            double dy = Y - other.Y;
            double dz = Z - other.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
        public Vector3Double Normalized()
        {
            double mag = magnitude;
            if (mag == 0)
                throw new InvalidOperationException("Cannot normalize a zero-length vector.");
            return new Vector3Double(X / mag, Y / mag, Z / mag);
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

        public static double Dot(Vector3Double a, Vector3Double b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vector3Double Cross(Vector3Double a, Vector3Double b)
        {
            return new Vector3Double(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X
            );
        }

        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }
    }

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

        public Vector4Double()
        {
            _values[0] = 0.0;
            _values[1] = 0.0;
            _values[2] = 0.0;
            _values[3] = 0.0;
        }
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

        public override string ToString()
        {
            return $"({X}, {Y}, {Z}, {W})";
        }
    }


}