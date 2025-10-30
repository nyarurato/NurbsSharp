using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Geometry
{
    public class NurbsCurve
    {
        public int Degree { get; set; }
        public KnotVector KnotVector { get; set; }
        public ControlPoint[] ControlPoints { get; set; }

        public NurbsCurve(int degree, KnotVector knotVector, ControlPoint[] controlPoints)
        {
            Degree = degree;
            KnotVector = knotVector;
            ControlPoints = controlPoints;
            checkValidity();
        }

        bool checkValidity()
        {
            int n = ControlPoints.Length;
            int m = KnotVector.Knots.Length;
            if (m != n + Degree + 1)
            {
                throw new InvalidOperationException("Invalid NURBS curve: knot vector length does not match control points and degree.");
            }
            return true;
        }

        /// <summary>
        /// Evaluate the NURBS curve at parameter u in [0, 1]
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public Vector3Double GetPos(int u)
        {
            var pos = CurveEvaluator.Evaluate(this, u);
            return new Vector3Double(pos.x, pos.y, pos.z);
        }
    }
}
