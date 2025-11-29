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
    }
}
