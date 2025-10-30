using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System.Numerics;
using System;
using System.Diagnostics;

namespace NurbsSharp.Evaluation
{
    
    public class CurveEvaluator
    {
        public static (double x,double y,double z) Evaluate(NurbsCurve curve, double u)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));
            if (curve.KnotVector.Knots[0] > u || curve.KnotVector.Knots[curve.KnotVector.Length-1] < u)
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' must be in the range [0, 1].");

            // ****** de Boor's algorithm *******
            int degree = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var controlPoints = curve.ControlPoints;
            int n = controlPoints.Length - 1;

            // find knot span index
            int k = -1;
            int s = 0; // multiplicity of u
            for (int i = 1; i < knots.Length; i++)
            {
                if (u >= knots[i-1] && u < knots[i])
                {
                    k = i-1;
                    break;
                }else if (u == knots[i])
                {
                    s++;
                }
            }
            if (u == knots[knots.Length - 1])
            {
                k = knots.Length - s -1;
            }


            //Console.WriteLine($"u={u}, knot span index k={k}");
            if (k == -1)
            {
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' is out of the knot vector range.");
            }

            Vector4Double[] d = new Vector4Double[degree + 1];
            for (int j = 0; j <= degree; j++)
            {
                var cp = controlPoints[k - degree + j].HomogeneousPosition;
                d[j] = cp;
            }

            // iterate
            for (int r = 1; r <= degree; r++)
            {
                for (int j = degree; j >= r; j--)
                {
                    double alpha= 0.0;
                    if (knots[j + 1 + k - r] != knots[k - degree + j])
                    {
                        alpha = (u - knots[k - degree + j]) / (knots[j + 1 + k - r] - knots[k - degree + j]);
                    }
                    //Console.WriteLine($"k={k} r={r}, j={j}, deg={degree}, alpha={alpha}, bunbo={j + 1 + k - r},{k-degree+j}-> {knots[j + 1 + k - r]} - {knots[k - degree + j]}");
                    d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
                    //Console.WriteLine($"  Updated d[{j}] = {d[j]}");
                }
            }
            for(int j = 0; j <= degree; j++)
            {
                //Console.WriteLine($"Final d[{j}] = {d[j]}");
            }
            // convert back to 3D
            Vector4Double resultH = d[degree];
            return (resultH.X / resultH.W, resultH.Y / resultH.W, resultH.Z / resultH.W);
        }


        public static double CurveLength(NurbsCurve curve, double start_u, double end_u)
        {
            if (curve == null)
                throw new ArgumentNullException(nameof(curve));
            
            double length = 0.0;

            return length;
        }
    }
}
