using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Geometry
{
    public class NurbsCurve:IGeometry
    {
        public int Degree { get; set; }
        public KnotVector KnotVector { get; set; }
        public ControlPoint[] ControlPoints { get; set; }

        public NurbsCurve(int degree, KnotVector knotVector, ControlPoint[] controlPoints)
        {
            Degree = degree;
            KnotVector = knotVector ?? throw new ArgumentNullException(nameof(knotVector));
            ControlPoints = controlPoints ?? throw new ArgumentNullException(nameof(controlPoints));
            Validate();
        }

        private void Validate()
        {
            int n = ControlPoints.Length;
            int m = KnotVector.Knots.Length;
            if (m != n + Degree + 1)
            {
                throw new InvalidOperationException("Invalid NURBS curve: knot vector length does not match control points and degree.");
            }
        }

        /// <summary>
        /// (en) Evaluates the position on the NURBS curve at the specified parameter u. The range is the same as the knot vector's minimum and maximum values.
        /// (ja) 指定したパラメータ u でNURBS曲線上の位置を評価します。レンジはノットベクトルの最小値と最大値と同じです。
        /// </summary>
        /// <param name="u"></param>
        /// <returns>Position Vector</returns>
        public Vector3Double GetPos(double u)
        {
            var pos = CurveEvaluator.Evaluate(this, u);
            return new Vector3Double(pos.x, pos.y, pos.z);
        }

        public double GetLength()
        {
            double start_u = KnotVector.Knots[0];
            double end_u = KnotVector.Knots[KnotVector.Length - 1];
            double epsilon = (end_u - start_u)/10000;

            double len = CurveEvaluator.CurveLength(this, start_u,end_u,epsilon );
            return len;
        }
    }
}
