using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Geometry
{
    /// <summary>
    /// (en) NURBS curve
    /// (ja) NURBS曲線
    /// </summary>
    public class NurbsCurve:IGeometry
    {
        /// <summary>
        /// (en) Degree of the NURBS curve
        /// (ja) NURBS曲線の次数
        /// </summary>
        public int Degree { get; private set; }

        /// <summary>
        /// (en) Knot vector of the NURBS curve
        /// (ja) NURBS曲線のノットベクトル
        /// </summary>

        public KnotVector KnotVector { get; private set; }

        /// <summary>
        /// (en) Control points of the NURBS curve
        /// (ja) NURBS曲線の制御点
        /// </summary>
        public ControlPoint[] ControlPoints { get; set; }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="degree"></param>
        /// <param name="knotVector"></param>
        /// <param name="controlPoints"></param>
        /// <exception cref="ArgumentNullException"></exception>
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

            if(n == 0)
            {
                throw new InvalidOperationException("Invalid NURBS curve: no control points defined.");
            }
            if (n < Degree + 1)
            {
                throw new InvalidOperationException($"Invalid NURBS curve: not enough control points for the given degree. ControlPoints{n} < Degree{Degree} + 1");
            }
            if (m != n + Degree + 1)
            {
                throw new InvalidOperationException($"Invalid NURBS curve: knot vector length does not match control points and degree. n{n} + p{Degree} + 1 != m{m}");
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
            return CurveEvaluator.Evaluate(this, u);
        }

        /// <summary>
        /// (en) Return the calulated length of the NURBS curve
        /// (ja) NURBS曲線の計算された長さを返します
        /// </summary>
        /// <returns></returns>
        public double GetLength()
        {
            double start_u = KnotVector.Knots[0];
            double end_u = KnotVector.Knots[KnotVector.Length - 1];

            double len = CurveEvaluator.CurveLength(this, start_u,end_u);
            return len;
        }
        /// <summary>
        /// (en) Return the calulated length of the NURBS curve between the specified parameters
        /// (ja) 指定したパラメータ間のNURBS曲線の計算された長さを返します
        /// </summary>
        /// <param name="start_u"></param>
        /// <param name="end_u"></param>
        /// <returns></returns>
        public double GetLength(double start_u, double end_u)
        {
            double len = CurveEvaluator.CurveLength(this, start_u, end_u);
            return len;
        }

        /// <summary>
        /// Returns a string that represents the current object.
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"NurbsCurve(Degree={Degree}, ControlPoints={ControlPoints.Length}, Knots={KnotVector.Length})";
        }
    }
}
