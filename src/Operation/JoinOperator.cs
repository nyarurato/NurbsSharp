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
    //TODO: add join with continuity options (G1, G2). it may be approximation like lofting.
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
            Vector3Double endPos1 = first_curve.GetPos(knots1[^1]); //estimate clamped knot
            Vector3Double startPos2 = second_curve.GetPos(knots2[0]);
            if (endPos1.DistanceTo(startPos2) > tolerance)
            {
                throw new InvalidOperationException("The end point of curve1 and the start point of curve2 are not coincident.");
            }

            // Check weight continuity at join point
            var cp1 = first_curve.ControlPoints;
            var cp2 = second_curve.ControlPoints;
            if (Math.Abs(cp1[^1].Weight - cp2[0].Weight) > tolerance)
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
            
            double max_u = knots1[^1]; //estimate clamped knot
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

        /// <summary>
        /// (en) Join two NURBS surfaces with C0 (positional) continuity along U or V direction.
        /// The edges of surface1 and surface2 must match at the join location.
        /// NOTE: This method does NOT guarantee C1 (tangent) continuity.
        /// (ja) 2つのNURBSサーフェスをU方向またはV方向のエッジに沿ってC0（位置）連続性で結合します。
        /// surface1とsurface2のエッジが結合位置で一致する必要があります。
        /// 注意: このメソッドはC1（接線）連続性を保証しません。
        /// </summary>
        /// <param name="surface1">First surface</param>
        /// <param name="surface2">Second surface</param>
        /// <param name="direction">Direction of join (U or V)</param>
        /// <param name="tolerance">Tolerance for edge matching</param>
        /// <returns>Joined NURBS surface</returns>
        /// <exception cref="ArgumentNullException">Thrown when surface1 or surface2 is null</exception>
        /// <exception cref="InvalidOperationException">Thrown when edges don't match or sizes are incompatible</exception>
        /// <remarks>
        /// (en) When joining surfaces of different degrees, the surface with the lower degree is automatically elevated.
        /// (ja) 次数の異なるサーフェスを結合する場合、低い次数のサーフェスが自動的に次数昇格されます。
        /// </remarks>
        public static NurbsSurface JoinSurfaces(NurbsSurface surface1, NurbsSurface surface2, SurfaceDirection direction, double tolerance = 1e-6)
        {
            Guard.ThrowIfNull(surface1, nameof(surface1));
            Guard.ThrowIfNull(surface2, nameof(surface2));

            if (direction == SurfaceDirection.U)
            {
                return JoinSurfacesU(surface1, surface2, tolerance);
            }
            else
            {
                return JoinSurfacesV(surface1, surface2, tolerance);
            }
        }

        /// <summary>
        /// (en) Join two surfaces in U direction (surface1's Umax to surface2's Umin)
        /// (ja) U方向で2つのサーフェスを結合（surface1のUmaxとsurface2のUmin）
        /// </summary>
        private static NurbsSurface JoinSurfacesU(NurbsSurface surface1, NurbsSurface surface2, double tolerance)
        {
            int degreeU1 = surface1.DegreeU;
            int degreeU2 = surface2.DegreeU;
            int degreeV1 = surface1.DegreeV;
            int degreeV2 = surface2.DegreeV;

            ControlPoint[][] cp1 = surface1.ControlPoints;
            ControlPoint[][] cp2 = surface2.ControlPoints;

            int nU1 = cp1.Length;
            int nV1 = cp1[0].Length;
            int nU2 = cp2.Length;
            int nV2 = cp2[0].Length;

            // Check that V dimensions match
            if (nV1 != nV2)
            {
                throw new InvalidOperationException(
                    $"V dimensions must match for U-direction join. Surface1 has {nV1} V control points, Surface2 has {nV2}.");
            }

            // Check V degree match (orthogonal direction must match)
            if (degreeV1 != degreeV2)
            {
                throw new InvalidOperationException(
                    $"V degrees must match for U-direction join. Surface1 has degree {degreeV1}, Surface2 has degree {degreeV2}.");
            }

            // Verify edges match by sampling points
            double[] vSamples = [0, 0.25, 0.5, 0.75, 1];
            var knots1U = surface1.KnotVectorU.Knots;
            var knots2U = surface2.KnotVectorU.Knots;
            double uMax1 = knots1U[^1];
            double uMin2 = knots2U[0];

            foreach (var v in vSamples)
            {
                var edge1 = surface1.GetPos(uMax1, v);
                var edge2 = surface2.GetPos(uMin2, v);
                if (edge1.DistanceTo(edge2) > tolerance)
                {
                    throw new InvalidOperationException(
                        $"Edges do not match at v={v}. Distance: {edge1.DistanceTo(edge2)}");
                }
            }

            // Join curves in U direction for each V index
            List<NurbsCurve> joinedCurves = [];

            for (int vIndex = 0; vIndex < nV1; vIndex++)
            {
                // Extract U-direction curves for this V index
                ControlPoint[] curve1CP = new ControlPoint[nU1];
                for (int uIndex = 0; uIndex < nU1; uIndex++)
                {
                    curve1CP[uIndex] = cp1[uIndex][vIndex];
                }

                ControlPoint[] curve2CP = new ControlPoint[nU2];
                for (int uIndex = 0; uIndex < nU2; uIndex++)
                {
                    curve2CP[uIndex] = cp2[uIndex][vIndex];
                }

                var curve1 = new NurbsCurve(surface1.DegreeU, surface1.KnotVectorU, curve1CP);
                var curve2 = new NurbsCurve(surface2.DegreeU, surface2.KnotVectorU, curve2CP);

                // Join the curves
                var joinedCurve = JoinCurves(curve1, curve2, tolerance);

                joinedCurves.Add(joinedCurve);
            }

            // Reconstruct surface from joined curves
            int newNU = joinedCurves[0].ControlPoints.Length;
            ControlPoint[][] newControlPoints = new ControlPoint[newNU][];
            for (int uIndex = 0; uIndex < newNU; uIndex++)
            {
                newControlPoints[uIndex] = new ControlPoint[nV1];
                for (int vIndex = 0; vIndex < nV1; vIndex++)
                {
                    newControlPoints[uIndex][vIndex] = joinedCurves[vIndex].ControlPoints[uIndex];
                }
            }

            return new NurbsSurface(
                joinedCurves[0].Degree,
                surface1.DegreeV,
                joinedCurves[0].KnotVector,
                surface1.KnotVectorV,
                newControlPoints);
        }

        /// <summary>
        /// (en) Join two surfaces in V direction (surface1's Vmax to surface2's Vmin)
        /// (ja) V方向で2つのサーフェスを結合（surface1のVmaxとsurface2のVmin）
        /// </summary>
        private static NurbsSurface JoinSurfacesV(NurbsSurface surface1, NurbsSurface surface2, double tolerance)
        {
            int degreeU1 = surface1.DegreeU;
            int degreeU2 = surface2.DegreeU;
            int degreeV1 = surface1.DegreeV;
            int degreeV2 = surface2.DegreeV;

            ControlPoint[][] cp1 = surface1.ControlPoints;
            ControlPoint[][] cp2 = surface2.ControlPoints;

            int nU1 = cp1.Length;
            int nV1 = cp1[0].Length;
            int nU2 = cp2.Length;
            int nV2 = cp2[0].Length;

            // Check that U dimensions match
            if (nU1 != nU2)
            {
                throw new InvalidOperationException(
                    $"U dimensions must match for V-direction join. Surface1 has {nU1} U control points, Surface2 has {nU2}.");
            }

            // Check U degree match (orthogonal direction must match)
            if (degreeU1 != degreeU2)
            {
                throw new InvalidOperationException(
                    $"U degrees must match for V-direction join. Surface1 has degree {degreeU1}, Surface2 has degree {degreeU2}.");
            }

            // Verify edges match by sampling points
            double[] uSamples = [0, 0.25, 0.5, 0.75, 1];
            var knots1V = surface1.KnotVectorV.Knots;
            var knots2V = surface2.KnotVectorV.Knots;
            double vMax1 = knots1V[^1];
            double vMin2 = knots2V[0];

            foreach (var u in uSamples)
            {
                var edge1 = surface1.GetPos(u, vMax1);
                var edge2 = surface2.GetPos(u, vMin2);
                if (edge1.DistanceTo(edge2) > tolerance)
                {
                    throw new InvalidOperationException(
                        $"Edges do not match at u={u}. Distance: {edge1.DistanceTo(edge2)}");
                }
            }

            // Join curves in V direction for each U index
            List<NurbsCurve> joinedCurves = [];

            for (int uIndex = 0; uIndex < nU1; uIndex++)
            {
                // Extract V-direction curves for this U index
                ControlPoint[] curve1CP = cp1[uIndex];
                ControlPoint[] curve2CP = cp2[uIndex];

                var curve1 = new NurbsCurve(surface1.DegreeV, surface1.KnotVectorV, curve1CP);
                var curve2 = new NurbsCurve(surface2.DegreeV, surface2.KnotVectorV, curve2CP);

                // Join the curves
                var joinedCurve = JoinCurves(curve1, curve2, tolerance);

                joinedCurves.Add(joinedCurve);
            }

            // Reconstruct surface from joined curves
            int newNV = joinedCurves[0].ControlPoints.Length;
            ControlPoint[][] newControlPoints = new ControlPoint[nU1][];
            for (int uIndex = 0; uIndex < nU1; uIndex++)
            {
                newControlPoints[uIndex] = joinedCurves[uIndex].ControlPoints;
            }

            return new NurbsSurface(
                surface1.DegreeU,
                joinedCurves[0].Degree,
                surface1.KnotVectorU,
                joinedCurves[0].KnotVector,
                newControlPoints);
        }
    }
}
