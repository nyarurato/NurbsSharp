using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for NURBS surfaces
    /// (ja) NURBSサーフェスの操作クラス
    /// </summary>
    public static class SurfaceOperator
    {
        /// <summary>
        /// (en) Extract an isocurve at a fixed U parameter (returns a curve along V)
        /// (ja) 固定されたUパラメータでアイソカーブを抽出します（V方向の曲線を返します）
        /// </summary>
        public static NurbsCurve ExtractIsoCurveU(NurbsSurface surface, double u)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            
            // Resulting curve properties
            int degree = surface.DegreeV;
            KnotVector knotVector = surface.KnotVectorV;
            int nV = surface.ControlPoints[0].Length;
            int nU = surface.ControlPoints.Length;

            ControlPoint[] newControlPoints = new ControlPoint[nV];

            // For each column j, evaluate the curve defined by column j at u
            for (int j = 0; j < nV; j++)
            {
                // Extract column j as control points
                ControlPoint[] columnCPs = new ControlPoint[nU];
                for (int i = 0; i < nU; i++)
                {
                    columnCPs[i] = surface.ControlPoints[i][j];
                }

                // Create a temporary curve for this column
                var columnCurve = new NurbsCurve(surface.DegreeU, surface.KnotVectorU, columnCPs);

                // Evaluate at u to get the new control point
                // We must use homogeneous evaluation to preserve weights correctly
                Vector4Double homoPos = CurveEvaluator.EvaluateHomogeneous(columnCurve, u);
                
                // Convert back to ControlPoint (Position + Weight)
                // Vector4Double is (wx, wy, wz, w)
                // ControlPoint expects Position (x,y,z) and Weight (w)
                // Position = (wx/w, wy/w, wz/w)
                
                if (homoPos.W == 0)
                     throw new InvalidOperationException("Weight is zero during isocurve extraction.");

                Vector3Double pos = new Vector3Double(homoPos.X / homoPos.W, homoPos.Y / homoPos.W, homoPos.Z / homoPos.W);
                newControlPoints[j] = new ControlPoint(pos, homoPos.W);
            }

            return new NurbsCurve(degree, knotVector, newControlPoints);
        }

        /// <summary>
        /// (en) Extract an isocurve at a fixed V parameter (returns a curve along U)
        /// (ja) 固定されたVパラメータでアイソカーブを抽出します（U方向の曲線を返します）
        /// </summary>
        public static NurbsCurve ExtractIsoCurveV(NurbsSurface surface, double v)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            // Resulting curve properties
            int degree = surface.DegreeU;
            KnotVector knotVector = surface.KnotVectorU;
            int nU = surface.ControlPoints.Length;
            int nV = surface.ControlPoints[0].Length;

            ControlPoint[] newControlPoints = new ControlPoint[nU];

            // For each row i, evaluate the curve defined by row i at v
            for (int i = 0; i < nU; i++)
            {
                // Extract row i as control points
                ControlPoint[] rowCPs = new ControlPoint[nV];
                for (int j = 0; j < nV; j++)
                {
                    rowCPs[j] = surface.ControlPoints[i][j];
                }

                // Create a temporary curve for this row
                var rowCurve = new NurbsCurve(surface.DegreeV, surface.KnotVectorV, rowCPs);

                // Evaluate at v
                Vector4Double homoPos = CurveEvaluator.EvaluateHomogeneous(rowCurve, v);

                if (homoPos.W == 0)
                     throw new InvalidOperationException("Weight is zero during isocurve extraction.");

                Vector3Double pos = new Vector3Double(homoPos.X / homoPos.W, homoPos.Y / homoPos.W, homoPos.Z / homoPos.W);
                newControlPoints[i] = new ControlPoint(pos, homoPos.W);
            }

            return new NurbsCurve(degree, knotVector, newControlPoints);
        }

        /// <summary>
        /// (en) Find the closest point on surface to a given 3D point using Newton-Raphson with multiple initial points
        /// (ja) 複数の初期点からNewton-Raphson法で指定された3D点に最も近いサーフェス上の点を検索
        /// </summary>
        /// <param name="surface">Target surface</param>
        /// <param name="target">Target 3D point</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <param name="gridDivisions">Number of grid divisions for initial point search (default: 5)</param>
        /// <returns>Tuple of (u parameter, v parameter, point on surface, distance to target)</returns>
        [Obsolete("Use Analysis.SurfaceAnalyzer.FindClosestPoint instead.")]
        public static (double u, double v, Vector3Double point, double distance) FindClosestPoint(
            NurbsSurface surface, 
            Vector3Double target,
            double tolerance = 1e-6,
            int gridDivisions = 5)
        {
            return Analysis.SurfaceAnalyzer.FindClosestPoint(surface, target, tolerance, gridDivisions);
        }

        /// <summary>
        /// (en) Find the closest point on surface from a single initial guess using Newton-Raphson (fast)
        /// (ja) 単一の初期推定値からNewton-Raphson法で最近接点を検索（高速）
        /// </summary>
        /// <param name="surface">Target surface</param>
        /// <param name="target">Target 3D point</param>
        /// <param name="initialU">Initial guess for U parameter</param>
        /// <param name="initialV">Initial guess for V parameter</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>Tuple of (u parameter, v parameter, point on surface, distance to target)</returns>
        [Obsolete("Use Analysis.SurfaceAnalyzer.FindClosestPoint instead.")]
        public static (double u, double v, Vector3Double point, double distance) FindClosestPoint(
            NurbsSurface surface, 
            Vector3Double target,
            double initialU,
            double initialV,
            double tolerance = 1e-6)
        {
            return Analysis.SurfaceAnalyzer.FindClosestPoint(surface, target, initialU, initialV, tolerance);
        }
    }
}
