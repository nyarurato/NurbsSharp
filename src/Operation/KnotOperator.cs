using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for knot operations on NURBS
    /// (ja) NURBSのノット操作のためのオペレーター
    /// </summary>
    public class KnotOperator
    {
        /// <summary>
        /// (en) Inserts a knot into the knot vector multiple times while preserving the shape
        /// (ja) 形状を保つようにノットベクトルに複数回ノットを挿入します
        /// </summary>
        /// <param name="degree">Curve degree</param>
        /// <param name="knots">Original knot vector</param>
        /// <param name="controlPoints">Original control points</param>
        /// <param name="u">Knot value to insert</param>
        /// <param name="times">Number of times to insert the knot</param>
        /// <returns>New knot vector and control points</returns>
        public static (double[] newKnots, ControlPoint[] newControlPoints) InsertKnot(int degree, double[] knots, ControlPoint[] controlPoints, double u, int times)
        {
            if (times <= 0)
                return (knots, controlPoints);
            Guard.ThrowIfNull(knots, nameof(knots));
            Guard.ThrowIfNull(controlPoints, nameof(controlPoints));
            if(degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));
            if(u < knots[0] || u > knots[^1])
                throw new ArgumentOutOfRangeException(nameof(u), "Knot value to insert is out of the knot vector range.");

            //TODO: Check if times exceeds the maximum multiplicity allowed

            int n = controlPoints.Length - 1;
            int span = BasicEvaluator.FindSpan(degree, knots, u);
            // Perform insertion one at a time
            double[] currentKnots = (double[])knots.Clone();
            ControlPoint[] currentControlPoints = (ControlPoint[])controlPoints.Clone();
            for (int r = 0; r < times; r++)
            {
                int currentN = currentControlPoints.Length - 1;
                span = BasicEvaluator.FindSpan(degree, currentKnots, u);

                // Create new arrays
                double[] newKnots = new double[currentKnots.Length + 1];
                ControlPoint[] newControlPoints = new ControlPoint[currentControlPoints.Length + 1];
                // Copy knots before insertion point
                Array.Copy(currentKnots, 0, newKnots, 0, span + 1);

                // Insert new knot
                newKnots[span + 1] = u;

                // Copy knots after insertion point
                Array.Copy(currentKnots, span + 1, newKnots, span + 2, currentKnots.Length - span - 1);
                // Compute new control points using Boehm's algorithm
                // Copy unaffected control points (before span - degree)
                for (int i = 0; i <= span - degree; i++)
                {
                    newControlPoints[i] = currentControlPoints[i];
                }
                // Compute affected control points
                for (int i = span - degree + 1; i <= span; i++)
                {
                    if(LinAlg.ApproxEqual(currentKnots[i + degree] , currentKnots[i]))// Prevent division by zero
                    {
                        newControlPoints[i] = currentControlPoints[i - 1];
                        continue;
                    }
                    double alpha = (u - currentKnots[i]) / (currentKnots[i + degree] - currentKnots[i]);

                    var P0 = currentControlPoints[i - 1].HomogeneousPosition;
                    var P1 = currentControlPoints[i].HomogeneousPosition;
                    var newP = (1.0 - alpha) * P0 + alpha * P1;
                    if (LinAlg.ApproxEqual(newP.W, 0.0))
                    {
                        throw new InvalidOperationException("Knot insertion resulted in a control point with zero weight.");
                    }

                    newControlPoints[i] = new ControlPoint(
                        newP.X / newP.W,
                        newP.Y / newP.W,
                        newP.Z / newP.W,
                        newP.W
                    );
                }
                // Copy unaffected control points (after span)
                for (int i = span + 1; i <= currentN + 1; i++)
                {
                    newControlPoints[i] = currentControlPoints[i - 1];
                }
                currentKnots = newKnots;
                currentControlPoints = newControlPoints;
            }
            return (currentKnots, currentControlPoints);
        }

        /// <summary>
        /// (en) Inserts a knot into the NURBS curve multiple times while preserving the shape
        /// (ja) 形状を保つようにNURBS曲線に複数回ノットを挿入します
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <param name="times"></param>
        /// <returns></returns>
        public static NurbsCurve InsertKnot(NurbsCurve curve, double u, int times)
        {
            var (newKnots, newControlPoints) = InsertKnot(curve.Degree, curve.KnotVector.Knots, curve.ControlPoints, u, times);
            return new NurbsCurve(curve.Degree, new KnotVector(newKnots, curve.Degree), newControlPoints);
        }

        //TODO: RemoveKnot method
        //TODO: RefineKnot method
    }
}
