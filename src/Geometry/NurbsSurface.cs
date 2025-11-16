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
        /// Returns a string that represents the current NURBS surface.
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"NurbsSurface(DegreeU={DegreeU}, DegreeV={DegreeV}, ControlPoints=({ControlPoints.Length} x {ControlPoints[0].Length}), KnotsU={KnotVectorU.Length}, KnotsV={KnotVectorV.Length})";
        }
    }
}
