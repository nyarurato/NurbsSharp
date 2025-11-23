using System;

namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Control point used in NURBS
    /// (ja) NURBSで使用される制御点
    /// </summary>
    public class ControlPoint
    {
        /// <summary>
        /// XYZ
        /// </summary>
        public Vector3Double Position { get; set; }
        /// <summary>
        /// Control point weight
        /// </summary>
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

        /// <summary>
        /// (en) Translate this control point by delta (in-place).
        /// (ja) この制御点を並進移動する（破壊的に Position を更新する）。
        /// </summary>
        /// <param name="delta">translation vector</param>
        /// <exception cref="ArgumentNullException"></exception>"
        public void Translate(Vector3Double delta)
        {
            //Guard.ThrowIfNull(delta, nameof(delta));
            // Position を新しいインスタンスに置き換えることで共有参照による副作用を避ける
            Position = new Vector3Double(
                Position.X + delta.X,
                Position.Y + delta.Y,
                Position.Z + delta.Z);
        }

        /// <summary>
        /// (en) Translate by components (in-place).
        /// (ja) 成分指定で並進移動する（破壊的）。
        /// </summary>
        public void Translate(double dx, double dy, double dz)
        {
            Position = new Vector3Double(
                Position.X + dx,
                Position.Y + dy,
                Position.Z + dz);
        }
    }
}
