using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Tesselation
{
    /// <summary>
    /// (en)Tessellator for NURBS curves
    /// (ja)NURBS曲線のテッセレーター
    /// </summary>
    public class CurveTessellator
    {
        /// <summary>
        /// (en)Tessellate NURBS curve into a list of points
        /// (ja)NURBS曲線を点のリストに分割する
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="numPoints"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static List<Vector3Double> Tessellate(NurbsCurve curve, int numPoints)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            if (numPoints < 2)
                throw new ArgumentOutOfRangeException(nameof(numPoints), "Number of points must be at least 2.");
            List<Vector3Double> points = [];
            double uStart = curve.KnotVector.Knots[curve.Degree];
            double uEnd = curve.KnotVector.Knots[curve.KnotVector.Knots.Length - curve.Degree - 1];
            for (int i = 0; i < numPoints; i++)
            {
                double u = uStart + (uEnd - uStart) * i / (numPoints - 1);
                var point = curve.GetPos(u);
                points.Add(point);
            }
            return points;
        }
    }
}
