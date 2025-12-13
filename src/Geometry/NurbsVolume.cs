using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Geometry
{
    /// <summary>
    /// Experimental
    /// (en) NURBS volume
    /// (ja) NURBSボリューム
    /// </summary>
    public class NurbsVolume:IGeometry
    {
        /// <summary>
        /// Degree in U direction
        /// </summary>
        public int DegreeU { get; set; }
        /// <summary>
        /// Degree in V direction
        /// </summary>
        public int DegreeV { get; set; }
        /// <summary>
        /// Degree in W direction
        /// </summary>
        public int DegreeW { get; set; }
        /// <summary>
        /// Knot vector in U direction
        /// </summary>
        public KnotVector KnotVectorU { get; set; }
        /// <summary>
        /// Knot vector in V direction
        /// </summary>
        public KnotVector KnotVectorV { get; set; }
        /// <summary>
        /// Knot vector in W direction
        /// </summary>
        public KnotVector KnotVectorW { get; set; }
        /// <summary>
        /// Control points grid [U][V][W]
        /// </summary>
        public ControlPoint[][][] ControlPoints { get; private set; }

        private BoundingBox? _boundingBox;
        /// <summary>
        /// Bounding box by control points
        /// </summary>
        public BoundingBox BoundingBox
        {
            get
            {
                _boundingBox ??= BoundingBox.FromControlPoints(ControlPoints);
                return _boundingBox.Value;
            }
        }

        /// <summary>
        /// Constructor
        /// </summary>
        public NurbsVolume()
        {
            DegreeU = 0;
            DegreeV = 0;
            DegreeW = 0;
            KnotVectorU = new KnotVector();
            KnotVectorV = new KnotVector();
            KnotVectorW = new KnotVector();
            ControlPoints = [];
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="degreeU"></param>
        /// <param name="degreeV"></param>
        /// <param name="degreeW"></param>
        /// <param name="knotVectorU"></param>
        /// <param name="knotVectorV"></param>
        /// <param name="knotVectorW"></param>
        /// <param name="controlPoints"></param>
        public NurbsVolume(int degreeU, int degreeV, int degreeW,
                           KnotVector knotVectorU, KnotVector knotVectorV, KnotVector knotVectorW,
                           ControlPoint[][][] controlPoints)
        {
            DegreeU = degreeU;
            DegreeV = degreeV;
            DegreeW = degreeW;
            KnotVectorU = knotVectorU;
            KnotVectorV = knotVectorV;
            KnotVectorW = knotVectorW;
            ControlPoints = controlPoints;
            Validate();
        }

        private void Validate()
        {
            int nU = ControlPoints.Length;
            int nV = ControlPoints[0].Length;
            int nW = ControlPoints[0][0].Length;
            int mU = KnotVectorU.Knots.Length;
            int mV = KnotVectorV.Knots.Length;
            int mW = KnotVectorW.Knots.Length;

            if (mU != nU + DegreeU + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: U knot vector length does not match control points and degree.");
            }
            if (mV != nV + DegreeV + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: V knot vector length does not match control points and degree.");
            }
            if (mW != nW + DegreeW + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: W knot vector length does not match control points and degree.");
            }
        }

        /// <summary>
        /// (en) Translate the whole volume by delta (in-place).
        /// (ja) ボリュームの全制御点を並進移動する（破壊的）。
        /// </summary>
        /// <param name="delta"></param>
        public void Translate(Vector3Double delta)
        {
            Operation.TransformOperator.TranslateInPlace(this, delta);
        }

        /// <summary>
        /// (en) Translate by components (in-place).
        /// (ja) 成分指定でボリュームを並進移動する（破壊的）。
        /// </summary>
        public void Translate(double dx, double dy, double dz)
        {
            Operation.TransformOperator.TranslateInPlace(this, dx, dy, dz);
        }

        /// <summary>
        /// (en) Rotate the volume around an axis (in-place).
        /// (ja) ボリュームを軸周りに回転する（破壊的）。
        /// </summary>
        /// <param name="axis">Rotation axis (will be normalized)</param>
        /// <param name="angle">Angle in radians</param>
        /// <param name="center">Center of rotation (default: origin)</param>
        public void Rotate(Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Operation.TransformOperator.RotateInPlace(this, axis, angle, center);
        }

        /// <summary>
        /// (en) Scale the volume uniformly (in-place).
        /// (ja) ボリュームを一様にスケールする（破壊的）。
        /// </summary>
        /// <param name="scale">Scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public void Scale(double scale, Vector3Double? center = null)
        {
            Operation.TransformOperator.ScaleInPlace(this, scale, center);
        }

        /// <summary>
        /// (en) Scale the volume non-uniformly (in-place).
        /// (ja) ボリュームを非一様にスケールする（破壊的）。
        /// </summary>
        /// <param name="sx">X scale factor</param>
        /// <param name="sy">Y scale factor</param>
        /// <param name="sz">Z scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public void Scale(double sx, double sy, double sz, Vector3Double? center = null)
        {
            Operation.TransformOperator.ScaleInPlace(this, sx, sy, sz, center);
        }

        /// <summary>
        /// (en) Transform the volume using a transformation matrix (in-place).
        /// (ja) 変換行列を使用してボリュームを変換する（破壊的）。
        /// </summary>
        /// <param name="matrix">Transformation matrix</param>
        public void Transform(Matrix4x4 matrix)
        {
            Operation.TransformOperator.TransformInPlace(this, matrix);
        }
    }
}
