using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Analysis;

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

            double len = CurveAnalyzer.CurveLength(this, start_u,end_u);
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
            double len = CurveAnalyzer.CurveLength(this, start_u, end_u);
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
            return CurveAnalyzer.EvaluateCurvature(this, u);
        }

        /// <summary>
        /// (en) Returns the tangent vector of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の接線ベクトルを返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public Vector3Double GetTangent(double u)
        {
            return CurveAnalyzer.EvaluateTangent(this,u);
        }

        /// <summary>
        /// (en) Returns the normal vector of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の法線ベクトルを返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public Vector3Double GetNormal(double u)
        {
            return CurveAnalyzer.EvaluateNormal(this, u);
        }

        /// <summary>
        /// (en) Returns tangent, normal of the NURBS curve at the specified parameter u
        /// (ja) 指定したパラメータ u でのNURBS曲線の接線、法線を返します
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public (Vector3Double tangent, Vector3Double normal) GetFrenetFrame(double u)
        {
            return CurveAnalyzer.EvaluateTangentNormal(this, u);
        }

        /// <summary>
        /// (en) Find the closest point on this curve to a given 3D point
        /// (ja) 指定された3D点に最も近い曲線上の点を検索します
        /// </summary>
        /// <param name="target">Target 3D point</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>Tuple of (t parameter, point on curve, distance to target)</returns>
        public (double t, Vector3Double point, double distance) FindClosestPoint(Vector3Double target, double tolerance = 1e-6)
        {
            return CurveAnalyzer.FindClosestPoint(this, target, tolerance: tolerance);
        }

        /// <summary>
        /// (en) Find the closest point on this curve from a single initial guess (faster if you have a good initial guess)
        /// (ja) 単一の初期推定値から最近接点を検索します（適切な初期値がある場合高速）
        /// </summary>
        /// <param name="target">Target 3D point</param>
        /// <param name="initialT">Initial guess for t parameter</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>Tuple of (t parameter, point on curve, distance to target)</returns>
        public (double t, Vector3Double point, double distance) FindClosestPointWithInitialGuess(Vector3Double target, double initialT, double tolerance = 1e-6)
        {
            return CurveAnalyzer.FindClosestPoint(this, target, initialT, tolerance);
        }

        /// <summary>
        /// (en) Translate the whole curve by delta (in-place).
        /// (ja) 曲線の全制御点を並進移動する（破壊的）。
        /// </summary>
        /// <param name="delta"></param>
        /// <exception cref="ArgumentNullException"></exception>"
        public void Translate(Vector3Double delta)
        {
            Operation.TransformOperator.TranslateInPlace(this, delta);
        }

        /// <summary>
        /// (en) Translate by components (in-place).
        /// (ja) 成分指定で曲線を並進移動する（破壊的）。
        /// </summary>
        public void Translate(double dx, double dy, double dz)
        {
            Operation.TransformOperator.TranslateInPlace(this, dx, dy, dz);
        }

        /// <summary>
        /// (en) Rotate the curve around an axis (in-place).
        /// (ja) 曲線を軸周りに回転する（破壊的）。
        /// </summary>
        /// <param name="axis">Rotation axis (will be normalized)</param>
        /// <param name="angle">Angle in radians</param>
        /// <param name="center">Center of rotation (default: origin)</param>
        public void Rotate(Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Operation.TransformOperator.RotateInPlace(this, axis, angle, center);
        }

        /// <summary>
        /// (en) Scale the curve uniformly (in-place).
        /// (ja) 曲線を一様にスケールする（破壊的）。
        /// </summary>
        /// <param name="scale">Scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public void Scale(double scale, Vector3Double? center = null)
        {
            Operation.TransformOperator.ScaleInPlace(this, scale, center);
        }

        /// <summary>
        /// (en) Scale the curve non-uniformly (in-place).
        /// (ja) 曲線を非一様にスケールする（破壊的）。
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
        /// (en) Transform the curve using a transformation matrix (in-place).
        /// (ja) 変換行列を使用して曲線を変換する（破壊的）。
        /// </summary>
        /// <param name="matrix">Transformation matrix</param>
        public void Transform(Matrix4x4 matrix)
        {
            Operation.TransformOperator.TransformInPlace(this, matrix);
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
