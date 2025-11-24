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
    /// (en) Direction for splitting NURBS surfaces
    /// (ja) NURBSサーフェスを分割する方向
    /// </summary>
    public enum SplitDirection
    {
        /// <summary>
        /// Split along U direction
        /// </summary>
        U,
        /// <summary>
        /// Split along V direction
        /// </summary>
        V
    }

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

        /// <summary>
        /// (en) Splits the NURBS surface at the specified parameter in U or V direction
        /// (ja) 指定したパラメータでNURBSサーフェスをU方向またはV方向に分割します
        /// </summary>
        /// <param name="surface">The surface to split</param>
        /// <param name="parameter">Parameter at which to split</param>
        /// <param name="direction">Direction to split (U or V)</param>
        /// <returns>Tuple of two NURBS surfaces (surface1, surface2)</returns>
        /// <exception cref="ArgumentNullException">Thrown when surface is null</exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown when parameter is out of range</exception>
        public static (NurbsSurface surface1, NurbsSurface surface2) SplitSurface(NurbsSurface surface, double parameter, SplitDirection direction)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            if (direction == SplitDirection.U)
            {
                return SplitSurfaceU(surface, parameter);
            }
            else
            {
                return SplitSurfaceV(surface, parameter);
            }
        }

        /// <summary>
        /// (en) Splits the NURBS surface in U direction
        /// (ja) NURBSサーフェスをU方向に分割します
        /// </summary>
        private static (NurbsSurface surface1, NurbsSurface surface2) SplitSurfaceU(NurbsSurface surface, double u)
        {
            // Validate parameter range
            double[] knotsU = surface.KnotVectorU.Knots;
            if (u < knotsU[0] || u > knotsU[^1])
            {
                throw new ArgumentOutOfRangeException(nameof(u), 
                    $"Parameter u={u} is out of range [{knotsU[0]}, {knotsU[^1]}]");
            }

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            KnotVector knotVectorV = surface.KnotVectorV;
            ControlPoint[][] controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;        // Number of rows in U direction
            int nV = controlPoints[0].Length;     // Number of columns in V direction

            // For each V index, extract U-direction curve and split it
            List<NurbsCurve> leftCurves = [];
            List<NurbsCurve> rightCurves = [];

            for (int vIndex = 0; vIndex < nV; vIndex++)
            {
                // Extract control points along U direction for this V index
                ControlPoint[] curveControlPoints = new ControlPoint[nU];
                for (int uIndex = 0; uIndex < nU; uIndex++)
                {
                    curveControlPoints[uIndex] = controlPoints[uIndex][vIndex];
                }

                // Create temporary curve along U direction
                var curve = new NurbsCurve(degreeU, surface.KnotVectorU, curveControlPoints);

                // Split the curve
                var (leftCurve, rightCurve) = SplitCurve(curve, u);

                leftCurves.Add(leftCurve);
                rightCurves.Add(rightCurve);
            }

            // Reconstruct two surfaces from split curves
            // Left surface
            int leftNU = leftCurves[0].ControlPoints.Length;
            ControlPoint[][] leftControlPoints = new ControlPoint[leftNU][];
            for (int uIndex = 0; uIndex < leftNU; uIndex++)
            {
                leftControlPoints[uIndex] = new ControlPoint[nV];
                for (int vIndex = 0; vIndex < nV; vIndex++)
                {
                    leftControlPoints[uIndex][vIndex] = leftCurves[vIndex].ControlPoints[uIndex];
                }
            }
            var leftSurface = new NurbsSurface(
                degreeU, 
                degreeV, 
                leftCurves[0].KnotVector, 
                knotVectorV, 
                leftControlPoints);

            // Right surface
            int rightNU = rightCurves[0].ControlPoints.Length;
            ControlPoint[][] rightControlPoints = new ControlPoint[rightNU][];
            for (int uIndex = 0; uIndex < rightNU; uIndex++)
            {
                rightControlPoints[uIndex] = new ControlPoint[nV];
                for (int vIndex = 0; vIndex < nV; vIndex++)
                {
                    rightControlPoints[uIndex][vIndex] = rightCurves[vIndex].ControlPoints[uIndex];
                }
            }
            var rightSurface = new NurbsSurface(
                degreeU, 
                degreeV, 
                rightCurves[0].KnotVector, 
                knotVectorV, 
                rightControlPoints);

            return (leftSurface, rightSurface);
        }

        /// <summary>
        /// (en) Splits the NURBS surface in V direction
        /// (ja) NURBSサーフェスをV方向に分割します
        /// </summary>
        private static (NurbsSurface surface1, NurbsSurface surface2) SplitSurfaceV(NurbsSurface surface, double v)
        {
            // Validate parameter range
            double[] knotsV = surface.KnotVectorV.Knots;
            if (v < knotsV[0] || v > knotsV[^1])
            {
                throw new ArgumentOutOfRangeException(nameof(v), 
                    $"Parameter v={v} is out of range [{knotsV[0]}, {knotsV[^1]}]");
            }

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            KnotVector knotVectorU = surface.KnotVectorU;
            ControlPoint[][] controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;        // Number of rows in U direction
            int nV = controlPoints[0].Length;     // Number of columns in V direction

            // For each U index, extract V-direction curve and split it
            List<NurbsCurve> leftCurves = [];
            List<NurbsCurve> rightCurves = [];

            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                // Extract control points along V direction for this U index
                ControlPoint[] curveControlPoints = controlPoints[uIndex];

                // Create temporary curve along V direction
                var curve = new NurbsCurve(degreeV, surface.KnotVectorV, curveControlPoints);

                // Split the curve
                var (leftCurve, rightCurve) = SplitCurve(curve, v);

                leftCurves.Add(leftCurve);
                rightCurves.Add(rightCurve);
            }

            // Reconstruct two surfaces from split curves
            // Left surface (lower V values)
            int leftNV = leftCurves[0].ControlPoints.Length;
            ControlPoint[][] leftControlPoints = new ControlPoint[nU][];
            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                leftControlPoints[uIndex] = leftCurves[uIndex].ControlPoints;
            }
            var leftSurface = new NurbsSurface(
                degreeU, 
                degreeV, 
                knotVectorU, 
                leftCurves[0].KnotVector, 
                leftControlPoints);

            // Right surface (higher V values)
            int rightNV = rightCurves[0].ControlPoints.Length;
            ControlPoint[][] rightControlPoints = new ControlPoint[nU][];
            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                rightControlPoints[uIndex] = rightCurves[uIndex].ControlPoints;
            }
            var rightSurface = new NurbsSurface(
                degreeU, 
                degreeV, 
                knotVectorU, 
                rightCurves[0].KnotVector, 
                rightControlPoints);

            return (leftSurface, rightSurface);
        }
    }
}

