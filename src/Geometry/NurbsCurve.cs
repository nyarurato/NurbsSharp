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
        private BoundingBox? _boundingBox;
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
        public ControlPoint[] ControlPoints { get; private set; }

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
        /// (en) Returns the curvature of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の曲率を返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public double GetCurvature(double u)
        {
            return CurveEvaluator.EvaluateCurvature(this, u);
        }

        /// <summary>
        /// (en) Returns the tangent vector of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の接線ベクトルを返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public Vector3Double GetTangent(double u)
        {
            return CurveEvaluator.EvaluateTangent(this,u);
        }

        /// <summary>
        /// (en) Returns the normal vector of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の法線ベクトルを返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public Vector3Double GetNormal(double u)
        {
            return CurveEvaluator.EvaluateNormal(this, u);
        }

        /// <summary>
        /// (en) Returns tangent, normal of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の接線、法線を返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public (Vector3Double tangent, Vector3Double normal) GetFrenetFrame(double u)
        {
            return CurveEvaluator.EvaluatTangentNormal(this, u);
        }

        /// <summary>
        /// (en) Translate the whole curve by delta (in-place).
        /// (ja) 曲線の全制御点を並進移動する（破壊的）。
        /// </summary>
        /// <param name="delta"></param>
        /// <exception cref="ArgumentNullException"></exception>"
        public void Translate(Vector3Double delta)
        {
            //Guard.ThrowIfNull(delta, nameof(delta));
            foreach (var cp in ControlPoints)
            {
                cp.Translate(delta);
            }
        }

        /// <summary>
        /// (en) Translate by components (in-place).
        /// (ja) 成分指定で曲線を並進移動する（破壊的）。
        /// </summary>
        public void Translate(double dx, double dy, double dz)
        {
            foreach (var cp in ControlPoints)
            {
                cp.Translate(dx,dy,dz);
            }
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
