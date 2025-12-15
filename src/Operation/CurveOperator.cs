using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for NURBS curves
    /// (ja) NURBS曲線の操作クラス
    /// </summary>
    public static class CurveOperator
    {
        /// <summary>
        /// (en) Find the closest point on curve to a given 3D point using Newton-Raphson with grid search for initial point
        /// (ja) グリッド探索とNewton-Raphson法で指定された3D点に最も近い曲線上の点を検索
        /// </summary>
        /// <param name="curve">Target curve</param>
        /// <param name="target">Target 3D point</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <param name="gridDivisions">Number of grid divisions for initial point search (default: 20)</param>
        /// <returns>Tuple of (t parameter, point on curve, distance to target)</returns>
        [Obsolete("Use Analysis.CurveAnalyzer.FindClosestPoint instead.")]
        public static (double t, Vector3Double point, double distance) FindClosestPoint(
            NurbsCurve curve,
            Vector3Double target,
            double tolerance = 1e-6,
            int gridDivisions = 20)
        {
            return Analysis.CurveAnalyzer.FindClosestPoint(curve, target, tolerance, gridDivisions);
        }

        /// <summary>
        /// (en) Find the closest point on curve from a single initial guess using Newton-Raphson (fast)
        /// (ja) 単一の初期推定値からNewton-Raphson法で最近接点を検索（高速）
        /// </summary>
        /// <param name="curve">Target curve</param>
        /// <param name="target">Target 3D point</param>
        /// <param name="initialT">Initial guess for t parameter</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <returns>Tuple of (t parameter, point on curve, distance to target)</returns>
        [Obsolete("Use Analysis.CurveAnalyzer.FindClosestPoint instead.")]
        public static (double t, Vector3Double point, double distance) FindClosestPoint(
            NurbsCurve curve,
            Vector3Double target,
            double initialT,
            double tolerance = 1e-6)
        {
            return Analysis.CurveAnalyzer.FindClosestPoint(curve, target, initialT, tolerance);
        }

    }
}
