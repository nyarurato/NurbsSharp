namespace NurbsSharp.Core
{
    public class ControlPoint
    {
        public Vector3Double Position { get; set; }
        public double Weight { get; set; }

        public ControlPoint()
        {
            Position = new Vector3Double(0.0, 0.0, 0.0);
            Weight = 1.0;
        }

        public ControlPoint(double x,double y,double z,double w) {
            Position = new Vector3Double(x, y, z);
            Weight = w;
        }

        public ControlPoint(Vector3Double position, double weight)
        {
            Position = position;
            Weight = weight;
        }

        public Vector4Double HomogeneousPosition
        {
            get
            {
                return new Vector4Double(
                    Position.X * Weight,
                    Position.Y * Weight,
                    Position.Z * Weight,
                    Weight);
            }
        }
    }
}
