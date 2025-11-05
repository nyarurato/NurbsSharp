namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Control point used in NURBS
    /// (ja) NURBSで使用される制御点
    /// </summary>
    public class ControlPoint
    {

        public Vector3Double Position { get; set; }
        public double Weight { get; set; }

        /// <summary>
        /// Constructor
        /// </summary>
        public ControlPoint()
        {
            Position = new Vector3Double(0.0, 0.0, 0.0);
            Weight = 1.0;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        public ControlPoint(double x, double y, double z)
        {
            Position = new Vector3Double(x, y, z);
            Weight = 1.0;
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="w"></param>
        public ControlPoint(double x,double y,double z,double w) {
            Position = new Vector3Double(x, y, z);
            Weight = w;
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="position"></param>
        /// <param name="weight"></param>
        public ControlPoint(Vector3Double position, double weight)
        {
            Position = position;
            Weight = weight;
        }

        /// <summary>
        /// return (x*w, y*w, z*w, w)
        /// </summary>
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
