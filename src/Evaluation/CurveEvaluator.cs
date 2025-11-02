using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System.Numerics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace NurbsSharp.Evaluation
{

    public class CurveEvaluator:BasicEvaluator
    {
        public static (double x, double y, double z) Evaluate(NurbsCurve curve, double u)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));
            if (curve.KnotVector.Knots[0] > u || curve.KnotVector.Knots[curve.KnotVector.Length - 1] < u)
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' must be in the range [0, 1].");
            if (curve.Degree < 1)
                throw new ArgumentException("Curve degree must be at least 1.", nameof(curve));

            // ****** de Boor's algorithm *******
            int degree = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;

            if(u < knots[degree])
                u = knots[degree];
            else if (u > knots[knots.Length - degree - 1])
                u = knots[knots.Length - degree - 1];

            int k = FindSpan(degree, knots, u);

            Vector4Double[][] d = new Vector4Double[degree + 1][];
            //initialize d
            for (int i = 0; i <= degree; i++)
            {
                d[i] = new Vector4Double[degree + 1];
            }
            
            // convert back to 3D
            Vector4Double resultH = DeBoor(degree, knots, k, controlPoints.Select(cp => cp.HomogeneousPosition).ToArray(), u);
            return (resultH.X / resultH.W, resultH.Y / resultH.W, resultH.Z / resultH.W);
        }

        // TODO: Optimize using different numerical integration methods
        public static double CurveLength(NurbsCurve curve, double start_u, double end_u, double epsilon = 0.001)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));

            double length = 0.0;
            Vector3Double prevPoint = new Vector3Double();
            bool isFirstPoint = true;

            for (double u = start_u; u <= end_u; u += epsilon)
            {
                var pos = Evaluate(curve, u);
                Vector3Double currentPoint = new Vector3Double(pos.x, pos.y, pos.z);
                if (isFirstPoint)
                {
                    prevPoint = currentPoint;
                    isFirstPoint = false;
                }
                else
                {
                    length += prevPoint.DistanceTo(currentPoint);
                    prevPoint = currentPoint;
                }
            }

            return length;
        }

    }
}
