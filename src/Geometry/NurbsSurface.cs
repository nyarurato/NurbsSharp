using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Geometry
{
    /// <summary>
    /// (en) NURBS surface
    /// (ja) NURBSサーフェス
    /// </summary>
    public class NurbsSurface:IGeometry
    {
        private BoundingBox? _boundingBox;

        /// <summary>
        /// Degree in U direction
        /// </summary>
        public int DegreeU { get; private set; }
        /// <summary>
        /// Degree in V direction
        /// </summary>
        public int DegreeV { get; private set; }
        /// <summary>
        /// Knot vector in U direction
        /// </summary>
        public KnotVector KnotVectorU { get; private set; }
        /// <summary>
        /// Knot vector in V direction
        /// </summary>
        public KnotVector KnotVectorV { get; private set; }
        /// <summary>
        /// Control points grid [U][V]  <br/>
        ///     V direction ->          <br/>
        /// U [[CP00, CP01, CP02, ...], <br/>
        /// d  [CP10, CP11, CP12, ...], <br/>
        /// i  [CP20, CP21, CP22, ...], <br/>
        /// r  [...   ...   ...  ...]] 
        /// </summary>
        public ControlPoint[][] ControlPoints { get; private set; }

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

            if(nU == 0 || nV == 0)
            {
                throw new InvalidOperationException($"Invalid NURBS surface: Control points grid is empty. Cp_u:{nU}, Cp_v:{nV}");
            }

            if(nU < DegreeU + 1)
            {
                throw new InvalidOperationException($"Invalid NURBS surface: Not enough control points in U direction for the given degree. Knot:{mU} !=  Cp:{nU} + p:{DegreeU} + 1");
            }
            if(nV < DegreeV + 1)
            {
                throw new InvalidOperationException($"Invalid NURBS surface: Not enough control points in V direction for the given degree. Knot:{mV} !=  Cp:{nV} + p:{DegreeV} + 1");
            }

            if (mU != nU + DegreeU + 1)
            {
                throw new InvalidOperationException($"Invalid NURBS surface: U knot vector length does not match control points and degree. Knot:{mU} !=  Cp:{nU} + p:{DegreeU} + 1");
            }
            if (mV != nV + DegreeV + 1)
            {
                throw new InvalidOperationException($"Invalid NURBS surface: V knot vector length does not match control points and degree. Knot:{mV} !=  Cp:{nV} + p:{DegreeV} + 1");
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
            return SurfaceEvaluator.Evaluate(this, u, v);
        }

        /// <summary>
        /// (en) Calculate the surface area of the NURBS Surface
        /// (ja) NURBSサーフェスの表面積を計算する
        /// </summary>
        /// <returns></returns>
        public double GetSurfaceArea()
        {
            double startU = KnotVectorU.Knots.First();
            double endU = KnotVectorU.Knots.Last();
            double startV = KnotVectorV.Knots.First();
            double endV = KnotVectorV.Knots.Last();
            return SurfaceEvaluator.SurfaceArea(this,startU,endU,startV,endV);
        }

        /// <summary>
        /// (en) Calculate the surface area of the NURBS Surface within the specified parameter range
        /// (ja) 指定したパラメータ範囲内のNURBSサーフェスの表面積を計算する
        /// </summary>
        /// <param name="startU"></param>
        /// <param name="endU"></param>
        /// <param name="startV"></param>
        /// <param name="endV"></param>
        /// <returns></returns>
        public double GetSurfaceArea(double startU, double endU, double startV, double endV)
        {
            return SurfaceEvaluator.SurfaceArea(this, startU, endU, startV, endV);
        }

        /// <summary>
        /// (en) Get tangent vector at the specified parameter (u,v)
        /// (ja) 指定したパラメータ(u,v)での接線ベクトルを取得する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public (Vector3Double tangentU,Vector3Double tangentV) GetTangent(double u, double v)
        {
            return SurfaceEvaluator.EvaluateTangents(this, u, v);
        }

        /// <summary>
        /// (en) Get normal vector at the specified parameter (u,v)
        /// (ja) 指定したパラメータ(u,v)での法線ベクトルを取得する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vector3Double GetNormal(double u, double v)
        {
            return SurfaceEvaluator.EvaluateNormal(this, u, v);
        }

        /// <summary>
        /// (en) Get mean curvature and Gaussian curvature at the specified parameter (u,v)
        /// (ja) 指定したパラメータ(u,v)での平均曲率とガウス曲率を取得する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public (double meanCurvature, double gaussianCurvature) GetMeanAndGaussianCurvatures(double u, double v)
        {
            return SurfaceEvaluator.EvaluateMeanAndGaussianCurvatures(this, u, v);
        }

        /// <summary>
        /// (en) Get mean curvature at the specified parameter (u,v)
        /// (ja) 指定したパラメータ(u,v)での平均曲率を取得する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public double GetMeanCurvature(double u, double v)
        {
            return SurfaceEvaluator.EvaluateMeanAndGaussianCurvatures(this,u,v).meanCurvature;
        }

        /// <summary>
        /// (en) Get Gaussian curvature at the specified parameter (u,v)
        /// (ja) 指定したパラメータ(u,v)でのガウス曲率を取得する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public double GetGaussianCurvature(double u, double v)
        {
            return SurfaceEvaluator.EvaluateMeanAndGaussianCurvatures(this, u, v).gaussianCurvature;
        }

        /// <summary>
        /// (en) Get principal curvatures at the specified parameter (u,v)
        /// (ja) 指定したパラメータ(u,v)での主曲率を取得する
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        public (double k1, double k2) GetPrincipalCurvatures(double u, double v)
        {
            return SurfaceEvaluator.EvaluatePrincipalCurvatures(this, u, v);
        }

        /// <summary>
        /// (en) Get isocurve at fixed U parameter (returns curve along V)
        /// (ja) 固定されたUパラメータでのアイソカーブを取得する（V方向の曲線を返す）
        /// </summary>
        /// <param name="u"></param>
        /// <returns></returns>
        public NurbsCurve GetIsoCurveU(double u)
        {
            return Operation.SurfaceOperator.ExtractIsoCurveU(this, u);
        }

        /// <summary>
        /// (en) Get isocurve at fixed V parameter (returns curve along U)
        /// (ja) 固定されたVパラメータでのアイソカーブを取得する（U方向の曲線を返す）
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public NurbsCurve GetIsoCurveV(double v)
        {
            return Operation.SurfaceOperator.ExtractIsoCurveV(this, v);
        }

        /// <summary>
        /// (en) Find the closest point on this surface to a given 3D point using grid search and Newton-Raphson
        /// (ja) グリッド探索とNewton-Raphson法で指定された3D点に最も近いサーフェス上の点を検索します
        /// </summary>
        /// <param name="target">Target 3D point</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <param name="gridDivisions">Number of grid divisions for initial point search (default: 5)</param>
        /// <returns>Tuple of (u parameter, v parameter, point on surface, distance to target)</returns>
        public (double u, double v, Vector3Double point, double distance) FindClosestPoint(Vector3Double target, double tolerance = 1e-6, int gridDivisions = 5)
        {
            return Operation.SurfaceOperator.FindClosestPoint(this, target, tolerance, gridDivisions);
        }

        /// <summary>
        /// (en) Find the closest point on this surface from a single initial guess (faster if you have a good initial guess)
        /// (ja) 単一の初期推定値から最近接点を検索します（適切な初期値がある場合高速）
        /// </summary>
        /// <param name="target">Target 3D point</param>
        /// <param name="initialU">Initial guess for U parameter</param>
        /// <param name="initialV">Initial guess for V parameter</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>Tuple of (u parameter, v parameter, point on surface, distance to target)</returns>
        public (double u, double v, Vector3Double point, double distance) FindClosestPointWithInitialGuess(Vector3Double target, double initialU, double initialV, double tolerance = 1e-6)
        {
            return Operation.SurfaceOperator.FindClosestPoint(this, target, initialU, initialV, tolerance);
        }

        /// <summary>
        /// (en) Translate the whole surface by delta (in-place).
        /// (ja) サーフェスの全制御点を並進移動する（破壊的）。
        /// </summary>
        /// <param name="delta"></param>
        /// <exception cref="ArgumentNullException"></exception>"
        public void Translate(Vector3Double delta)
        {
            //Guard.ThrowIfNull(delta, nameof(delta));
            foreach (var row in ControlPoints)
            {
                foreach(var cp in row)
                {
                    cp.Translate(delta);
                }
            }
        }

        /// <summary>
        /// (en) Translate by components (in-place).
        /// (ja) 成分指定でサーフェスを並進移動する（破壊的）。
        /// </summary>
        public void Translate(double dx, double dy, double dz)
        {
            foreach (var row in ControlPoints)
            {
                foreach (var cp in row)
                {
                    cp.Translate(dx,dy,dz);
                }
            }
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
