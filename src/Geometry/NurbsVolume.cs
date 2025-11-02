using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Geometry
{
    public class NurbsVolume:IGeometry
    {
        public int DegreeU { get; set; }
        public int DegreeV { get; set; }
        public int DegreeW { get; set; }
        public KnotVector KnotVectorU { get; set; }
        public KnotVector KnotVectorV { get; set; }
        public KnotVector KnotVectorW { get; set; }
        public ControlPoint[][][] ControlPoints { get; set; }

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
            Console.WriteLine($"nU={nU}, nV={nV}, nW={nW}, mU={mU}, mV={mV}, mW={mW} ,DegreeU={DegreeU}, DegreeV={DegreeV}, Degree={DegreeW} ");
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
