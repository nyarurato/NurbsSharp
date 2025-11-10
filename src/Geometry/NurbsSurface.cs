using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;

namespace NurbsSharp.Geometry
{
    /// <summary>
    /// (en) NURBS surface
    /// (ja) NURBSサーフェス
    /// </summary>
    public class NurbsSurface:IGeometry
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
        /// Knot vector in U direction
        /// </summary>
        public KnotVector KnotVectorU { get; set; }
        /// <summary>
        /// Knot vector in V direction
        /// </summary>
        public KnotVector KnotVectorV { get; set; }
        /// <summary>
        /// Control points grid [U][V]
        /// </summary>
        public ControlPoint[][] ControlPoints { get; set; }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="degreeU"></param>
        /// <param name="degreeV"></param>
        /// <param name="knotVectorU"></param>
        /// <param name="knotVectorV"></param>
        /// <param name="controlPoints"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public NurbsSurface(int degreeU, int degreeV, KnotVector knotVectorU, KnotVector knotVectorV, ControlPoint[][] controlPoints)
        {
            DegreeU = degreeU;
            DegreeV = degreeV;
            KnotVectorU = knotVectorU ?? throw new ArgumentNullException(nameof(knotVectorU));
            KnotVectorV = knotVectorV ?? throw new ArgumentNullException(nameof(knotVectorV));
            ControlPoints = controlPoints ?? throw new ArgumentNullException(nameof(controlPoints));
            Validate();
        }

        private void Validate()
        {
            int nU = ControlPoints.Length;
            int nV = ControlPoints[0].Length;
            int mU = KnotVectorU.Knots.Length;
            int mV = KnotVectorV.Knots.Length;

            if (mU != nU + DegreeU + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: U knot vector length does not match control points and degree.");
            }
            if (mV != nV + DegreeV + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: V knot vector length does not match control points and degree.");
            }
        }

        /// <summary>
        /// (en) Evaluate the position on the NURBS Surface
        /// (ja) NURBSサーフェス上の位置を評価する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vector3Double GetPos(double u, double v)
        {
            var pos = Evaluation.SurfaceEvaluator.Evaluate(this, u, v);
            return new Vector3Double(pos.x, pos.y, pos.z);
        }

        /// <summary>
        /// Returns a string that represents the current NURBS surface.
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"NurbsSurface(DegreeU={DegreeU}, DegreeV={DegreeV}, ControlPoints=({ControlPoints.Length} x {ControlPoints[0].Length}), KnotsU={KnotVectorU.Length}, KnotsV={KnotVectorV.Length})";
        }
    }
}
