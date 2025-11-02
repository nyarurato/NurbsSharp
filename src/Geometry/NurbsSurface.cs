using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;

namespace NurbsSharp.Geometry
{
    public　class NurbsSurface:IGeometry
    {
        public int DegreeU { get; set; }
        public int DegreeV { get; set; }
        public KnotVector KnotVectorU { get; set; }
        public KnotVector KnotVectorV { get; set; }

        public ControlPoint[][] ControlPoints { get; set; }

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
            Console.WriteLine($"nU={nU}, nV={nV}, mU={mU}, mV={mV}, DegreeU={DegreeU}, DegreeV={DegreeV}");
            if (mU != nU + DegreeU + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: U knot vector length does not match control points and degree.");
            }
            if (mV != nV + DegreeV + 1)
            {
                throw new InvalidOperationException("Invalid NURBS surface: V knot vector length does not match control points and degree.");
            }
        }

        public Vector3Double GetPos(double u, double v)
        {
            var pos = Evaluation.SurfaceEvaluator.Evaluate(this, u, v);
            return new Vector3Double(pos.x, pos.y, pos.z);
        }
    }
}
