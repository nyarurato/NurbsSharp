using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System.Numerics;
using System;
using System.Diagnostics;

namespace NurbsSharp.Evaluation
{

    public class CurveEvaluator
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

            // find knot span index
            int k = -1;
            int s = 0; // multiplicity of u
            if (u < knots[degree])
            {
                k = degree;
            }
            else
            {
                for (int i = degree; i < (knots.Length - 1); i++)
                {
                    if (u >= knots[i] && u < knots[i + 1])
                    {
                        k = i;
                        break;
                    }
                    else if (u == knots[i + 1])
                    {
                        s++;
                    }
                }
                if (u == knots[knots.Length - 1])
                {
                    k = knots.Length - s - 1;
                }
            }

            //Console.WriteLine($"u={u}, knot span index k={k}");
            if (k == -1)
            {
                throw new ArgumentOutOfRangeException(nameof(u), "Parameter 'u' is out of the knot vector range.");
            }

            Vector4Double[][] d = new Vector4Double[degree + 1][];
            //initialize d
            for (int i = 0; i <= degree; i++)
            {
                d[i] = new Vector4Double[degree + 1];
            }

            for (int j = 0; j <= degree; j++)
            {
                d[0][j] = controlPoints[k - degree + j].HomogeneousPosition;
            }
            //Console.WriteLine("Initial d values:");

            // iterate
            for (int r = 1; r <= degree; r++)
            {
                for (int j = degree; j >= r; j--)
                {
                    double alpha = 0.0;
                    if (knots[j + 1 + k - r] != knots[k - degree + j])
                    {
                        alpha = (u - knots[k - degree + j]) / (knots[j + 1 + k - r] - knots[k - degree + j]);
                    }
                    //Console.WriteLine($"k={k} r={r}, j={j}, deg={degree}, alpha={alpha}, bunbo={j + 1 + k - r},{k-degree+j}-> {knots[j + 1 + k - r]} - {knots[k - degree + j]}");
                    d[r][j] = (1.0 - alpha) * d[r - 1][j - 1] + alpha * d[r - 1][j];
                    //Console.WriteLine($"  Updated d[{r}][{j}] = {d[r][j]}");
                }
            }
            // convert back to 3D
            Vector4Double resultH = d[degree][degree];
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
