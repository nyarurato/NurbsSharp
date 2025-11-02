using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Tesselation
{
    public class CurveTessellator
    {
        public static List<Vector3Double> Tessellate(NurbsCurve curve, int numPoints)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));
            if (numPoints < 2)
                throw new ArgumentOutOfRangeException(nameof(numPoints), "Number of points must be at least 2.");
            List<Vector3Double> points = new List<Vector3Double>();
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
