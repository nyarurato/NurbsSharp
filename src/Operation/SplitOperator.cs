using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Evaluation;
using NurbsSharp.Operation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for splitting NURBS
    /// (ja) NURBSを分割するためのオペレーター
    /// </summary>
    public class SplitOperator
    {
        /// <summary>
        /// (en) Splits the NURBS curve at the specified parameter u using knot insertion algorithm
        /// (ja) ノット挿入アルゴリズムを使用して、指定したパラメータ u でNURBS曲線を分割します
        /// </summary>
        /// <param name="curve">The curve to split</param>
        /// <param name="u">Parameter at which to split</param>
        /// <returns>Tuple of two NURBS curves (left curve, right curve)</returns>
        public static (NurbsCurve leftCurve, NurbsCurve rightCurve) SplitCurve(NurbsCurve curve, double u)
        {
            int degree = curve.Degree;
            double[] knots = curve.KnotVector.Knots;
            ControlPoint[] controlPoints = curve.ControlPoints;
            // Insert knot u until it has multiplicity = degree + 1
            int span = BasicEvaluator.FindSpan(degree, knots, u);

            // Calculate current multiplicity of u in the knot vector
            int multiplicity = 0;
            for (int i = 0; i < knots.Length; i++)
            {
                if (LinAlg.ApproxEqual(knots[i], u))
                    multiplicity++;
            }
            // Number of times we need to insert u
            int timesToInsert = degree + 1 - multiplicity;
            // Perform knot insertion
            var (newKnots, newControlPoints) = KnotOperator.InsertKnot(degree, knots, controlPoints, u, timesToInsert);
            // Find the split point in the new knot vector
            int splitIndex = -1;
            for (int i = 0; i < newKnots.Length; i++)
            {
                if (LinAlg.ApproxEqual(newKnots[i], u))
                {
                    splitIndex = i;
                    break;
                }
            }

            // Create left curve: from start to split point
            int leftKnotCount = splitIndex + degree + 1;
            double[] leftKnots = new double[leftKnotCount];
            Array.Copy(newKnots, 0, leftKnots, 0, leftKnotCount);

            int leftControlPointCount = leftKnotCount - degree - 1;
            ControlPoint[] leftControlPoints = new ControlPoint[leftControlPointCount];
            Array.Copy(newControlPoints, 0, leftControlPoints, 0, leftControlPointCount);
            // Create right curve: from split point to end
            int rightKnotCount = newKnots.Length - splitIndex;
            double[] rightKnots = new double[rightKnotCount];
            Array.Copy(newKnots, splitIndex, rightKnots, 0, rightKnotCount);

            int rightControlPointCount = rightKnotCount - degree - 1;
            ControlPoint[] rightControlPoints = new ControlPoint[rightControlPointCount];
            Array.Copy(newControlPoints, splitIndex, rightControlPoints, 0, rightControlPointCount);
            var leftCurve = new NurbsCurve(degree, new KnotVector(leftKnots, degree), leftControlPoints);
            var rightCurve = new NurbsCurve(degree, new KnotVector(rightKnots, degree), rightControlPoints);
            return (leftCurve, rightCurve);
        }

        //TODO: SplitSurface method
    }
}
