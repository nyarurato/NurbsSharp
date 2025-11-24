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
    /// (en) Operator for joining NURBS
    /// (ja) NURBSを結合するためのオペレーター
    /// </summary>
    public class JoinOperator
    {
        /// <summary>
        /// (en) Join two NURBS curves with C0 (positional) continuity.
        /// The end point of curve1 must match the start point of curve2.
        /// NOTE: This method does NOT guarantee C1 (tangent) continuity.
        /// (ja) 2つのNURBS曲線をC0（位置）連続性で結合します。
        /// curve1の終点はcurve2の始点と一致する必要があります。
        /// 注意: このメソッドはC1（接線）連続性を保証しません。
        /// </summary>
        /// <param name="curve1">First curve</param>
        /// <param name="curve2">Second curve</param>
        /// <param name="tolerance">Tolerance for end point matching and weight comparison</param>
        /// <returns>Joined NURBS curve</returns>
        public static NurbsCurve JoinCurves(NurbsCurve curve1, NurbsCurve curve2, double tolerance = 1e-6)
        {

            // Join two NURBS curves. Join at the end of curve1 and the start of curve2.
            Guard.ThrowIfNull(curve1, nameof(curve1));
            Guard.ThrowIfNull(curve2, nameof(curve2));

            NurbsCurve first_curve = curve1;
            NurbsCurve second_curve = curve2;

            // Check G0 continuity (position)
            var knots1 = first_curve.KnotVector.Knots;
            var knots2 = second_curve.KnotVector.Knots;
            Vector3Double endPos1 = first_curve.GetPos(knots1[knots1.Length - 1]); //estimate clamped knot
            Vector3Double startPos2 = second_curve.GetPos(knots2[0]);
            if (endPos1.DistanceTo(startPos2) > tolerance)
            {
                throw new InvalidOperationException("The end point of curve1 and the start point of curve2 are not coincident.");
            }

            // Check weight continuity at join point
            var cp1 = first_curve.ControlPoints;
            var cp2 = second_curve.ControlPoints;
            if (Math.Abs(cp1[cp1.Length - 1].Weight - cp2[0].Weight) > tolerance)
            {
                throw new InvalidOperationException("The weights at the join point do not match.");
            }

            int degree1 = first_curve.Degree;
            int degree2 = second_curve.Degree;

            if (degree1 != degree2)
            {
                // degree elevation needed
                var targetDegree = Math.Max(degree1, degree2);
                if (degree1 < targetDegree)
                {
                    first_curve = DegreeOperator.ElevateDegree(first_curve, targetDegree - degree1);
                    cp1 = first_curve.ControlPoints; // Update reference after elevation
                    knots1 = first_curve.KnotVector.Knots;
                }
                else if (degree2 < targetDegree)
                {
                    second_curve = DegreeOperator.ElevateDegree(second_curve, targetDegree - degree2);
                    cp2 = second_curve.ControlPoints; // Update reference after elevation
                    knots2 = second_curve.KnotVector.Knots;
                }
            }
            // Now both curves have the same degree
            int degree = first_curve.Degree;

            // Merge Control Points
            // curve1: 0 ... n1
            // curve2: 0 ... n2
            // result: 0 ... n1, 1 ... n2 (skip curve2[0] because curve2[0] == curve1[n1])
            var newControlPoints = new ControlPoint[cp1.Length + cp2.Length - 1];
            Array.Copy(cp1, 0, newControlPoints, 0, cp1.Length);
            Array.Copy(cp2, 1, newControlPoints, cp1.Length, cp2.Length - 1);

            // Merge Knot Vectors
            // curve1 knots: u_0 ... u_m1
            // curve2 knots: v_0 ... v_m2
            // result: u_0 ... u_{m1-1}, (v_{p+1} + shift) ... (v_{m2} + shift)
            // where shift = u_{m1} - v_{0} (assuming standard clamped, v_0 = v_p)
            // For clamped knot vectors, max_u and min_v are at the ends
            
            double max_u = knots1[knots1.Length - 1]; //estimate clamped knot
            double min_v = knots2[0];
            double shift = max_u - min_v;
                        
            // The knot at u_max should have multiplicity p to ensure C0 continuity.
            // In CLAMPED knot vector, u_max has multiplicity p+1.
            // take all knots of curve1 except the last one
            // curve1 (p=2): 0,0,0, 1,1,1 (3 CPs)
            // curve2 (p=2): 0,0,0, 1,1,1 (3 CPs)
            // knots1: 0,0,0,1,1,1. Remove last -> 0,0,0,1,1
            // knots2: 0,0,0,1,1,1. Remove first p+1 (3) -> 1,1,1. Shift by (1-0)=1 -> 2,2,2
            // Result: 0,0,0,1,1, 2,2,2. Length = 8.

            var newKnotsList = new List<double>();
            
            // Add all knots from curve1 except the last one
            for (int i = 0; i < knots1.Length - 1; i++)
            {
                newKnotsList.Add(knots1[i]);
            }

            // Add knots from curve2 starting from p+1, shifted
            for (int i = degree + 1; i < knots2.Length; i++)
            {
                newKnotsList.Add(knots2[i] + shift);
            }

            var newKnotVector = new KnotVector(newKnotsList.ToArray(), degree);

            return new NurbsCurve(degree, newKnotVector, newControlPoints);
        }

        // TODO: JoinSurfaces method can be implemented similarly, handling U and V directions.
    }
}
