namespace NurbsSharp.Core
{
    public class Vector3Double
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public Vector3Double()
        {
            X = 0.0;
            Y = 0.0;
            Z = 0.0;
        }
        public Vector3Double(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
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
        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }
        
    }

    public class Vector4Double
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double W { get; set; }
        public Vector4Double()
        {
            X = 0.0;
            Y = 0.0;
            Z = 0.0;
            W = 0.0;
        }
        public Vector4Double(double x, double y, double z, double w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
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

        public override string ToString()
        {
            return $"({X}, {Y}, {Z}, {W})";
        }
    }


}
