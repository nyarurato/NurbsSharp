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
        public ControlPoint[][][] ControlPoints { get; set; }

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
            ControlPoints = new ControlPoint[0][][];
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
    }
}
