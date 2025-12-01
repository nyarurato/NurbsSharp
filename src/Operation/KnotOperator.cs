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
            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));
            if (u < knots[0] || u > knots[^1])
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
                    if (LinAlg.ApproxEqual(currentKnots[i + degree], currentKnots[i]))// Prevent division by zero
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

        /// <summary>
        /// (en) Inserts a knot into the NURBS surface multiple times in the specified direction while preserving the shape
        /// (ja) 形状を保つようにNURBSサーフェスに指定方向で複数回ノットを挿入します
        /// </summary>
        /// <param name="surface">Target NURBS surface</param>
        /// <param name="u">Knot value to insert</param>
        /// <param name="times">Number of times to insert the knot</param>
        /// <param name="direction">Direction (U or V) in which to insert the knot</param>
        /// <returns>New NURBS surface with the knot inserted</returns>
        public static NurbsSurface InsertKnot(NurbsSurface surface, double u, int times, SurfaceDirection direction)
        {
            if (direction == SurfaceDirection.U)
            {
                // Insert knot in U direction: treat each V-column as an independent curve
                int nV = surface.ControlPoints[0].Length;
                int oldNU = surface.ControlPoints.Length;

                // Extract control points for each V column and perform knot insertion
                List<ControlPoint[]> newColumns = new List<ControlPoint[]>();
                double[]? newKnots = null;

                for (int j = 0; j < nV; j++)
                {
                    // Extract control points for this V column (varying U)
                    ControlPoint[] columnCPs = new ControlPoint[oldNU];
                    for (int i = 0; i < oldNU; i++)
                    {
                        columnCPs[i] = surface.ControlPoints[i][j];
                    }

                    // Insert knot for this column
                    var (colNewKnots, colNewControlPoints) = InsertKnot(surface.DegreeU, surface.KnotVectorU.Knots, columnCPs, u, times);
                    newKnots = colNewKnots; // Same for all columns
                    newColumns.Add(colNewControlPoints);
                }

                // Rebuild control points 2D array [U][V]
                int newNU = newColumns[0].Length;
                ControlPoint[][] newCPs = new ControlPoint[newNU][];
                for (int i = 0; i < newNU; i++)
                {
                    newCPs[i] = new ControlPoint[nV];
                    for (int j = 0; j < nV; j++)
                    {
                        newCPs[i][j] = newColumns[j][i];
                    }
                }

                if (newKnots == null)
                    throw new InvalidOperationException("Failed to insert knot: no columns processed.");

                return new NurbsSurface(surface.DegreeU, surface.DegreeV, new KnotVector(newKnots, surface.DegreeU), surface.KnotVectorV, newCPs);
            }
            else // V direction
            {
                // Insert knot in V direction: treat each U-row as an independent curve
                int nU = surface.ControlPoints.Length;
                int oldNV = surface.ControlPoints[0].Length;

                // Extract control points for each U row and perform knot insertion
                ControlPoint[][] newCPs = new ControlPoint[nU][];
                double[]? newKnots = null;

                for (int i = 0; i < nU; i++)
                {
                    // Extract control points for this U row (varying V)
                    ControlPoint[] rowCPs = surface.ControlPoints[i]; // Already an array in V direction

                    // Insert knot for this row
                    var (rowNewKnots, rowNewControlPoints) = InsertKnot(surface.DegreeV, surface.KnotVectorV.Knots, rowCPs, u, times);
                    newKnots = rowNewKnots; // Same for all rows
                    newCPs[i] = rowNewControlPoints;
                }

                if (newKnots == null)
                    throw new InvalidOperationException("Failed to insert knot: no rows processed.");

                return new NurbsSurface(surface.DegreeU, surface.DegreeV, surface.KnotVectorU, new KnotVector(newKnots, surface.DegreeV), newCPs);
            }
        }

        /// <summary>
        /// (en) Removes a knot from the knot vector while preserving the curve shape within a tolerance.
        /// (ja) 許容誤差内で曲線の形状を保ちながら、ノットベクトルからノットを削除します
        /// </summary>
        /// <param name="degree">Curve degree</param>
        /// <param name="knots">Original knot vector</param>
        /// <param name="controlPoints">Original control points</param>
        /// <param name="u">Knot value to remove</param>
        /// <param name="tolerance">Maximum allowed deviation (default: 1e-6)</param>
        /// <returns>Tuple of (success, new knots, new control points). Success is false if removal would exceed tolerance.</returns>
        /// <remarks>
        /// The knot can only be removed if the curve shape deviation is within the specified tolerance.
        /// </remarks>
        public static (bool success, double[] newKnots, ControlPoint[] newControlPoints) RemoveKnot(int degree, double[] knots, ControlPoint[] controlPoints, double u, double tolerance = 1e-6)
        {
            Guard.ThrowIfNull(knots, nameof(knots));
            Guard.ThrowIfNull(controlPoints, nameof(controlPoints));
            Guard.ThrowIfNegative(degree, nameof(degree));

            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));
            if (u < knots[0] || u > knots[^1])
                throw new ArgumentOutOfRangeException(nameof(u), "Knot value is out of range.");

            // Find multiplicity of the knot to remove
            int s = GetMultiplicity(knots, u);
            if (s == 0)
                return (false, knots, controlPoints); // Knot doesn't exist

            // Find the knot span
            int r = BasicEvaluator.FindSpan(degree, knots, u);

            int n = controlPoints.Length - 1;
            int m = knots.Length - 1;

            // Check if this is an endpoint knot - cannot remove clamped endpoints
            bool isStartEndpoint = LinAlg.ApproxEqual(u, knots[0]);
            bool isEndEndpoint = LinAlg.ApproxEqual(u, knots[^1]);

            if ((isStartEndpoint || isEndEndpoint) && s == degree + 1)
            {
                return (false, knots, controlPoints);
            }

            int p = degree;
            int ord = p + 1;
            int first = r - p;
            int last = r - s;

            int off = first - 1; // offset for temp array
            ControlPoint[] temp = new ControlPoint[2 * p + 1];

            // Initialize temp array
            temp[0] = controlPoints[off];
            temp[last - off + 1] = controlPoints[last + 1];
            
            int i = first;
            int j = last;
            int ii = 1;
            int jj = last - off;

            // Compute new control points for one removal step
            while (j - i > 0)
            {
                double alpha_i = (u - knots[i]) / (knots[i + ord] - knots[i]);
                double alpha_j = (u - knots[j]) / (knots[j + ord] - knots[j]);

                var P_i = controlPoints[i].HomogeneousPosition;
                var temp_ii_minus_1 = temp[ii - 1].HomogeneousPosition;
                var newP_i = (P_i - (1.0 - alpha_i) * temp_ii_minus_1) / alpha_i;

                var P_j = controlPoints[j].HomogeneousPosition;
                var temp_jj_plus_1 = temp[jj + 1].HomogeneousPosition;
                var newP_j = (P_j - alpha_j * temp_jj_plus_1) / (1.0 - alpha_j);

                if (LinAlg.ApproxEqual(newP_i.W, 0.0) || LinAlg.ApproxEqual(newP_j.W, 0.0))
                    return (false, knots, controlPoints);

                temp[ii] = new ControlPoint(newP_i.X / newP_i.W, newP_i.Y / newP_i.W, newP_i.Z / newP_i.W, newP_i.W);
                temp[jj] = new ControlPoint(newP_j.X / newP_j.W, newP_j.Y / newP_j.W, newP_j.Z / newP_j.W, newP_j.W);

                i++;
                j--;
                ii++;
                jj--;
            }

            // Check if the knot is removable
            if (j - i < 0)
            {
                double dist = temp[ii - 1].Position.DistanceTo(temp[jj + 1].Position);
                if (dist > tolerance)
                    return (false, knots, controlPoints);
            }
            else if (j - i >= 0 && ii < temp.Length && temp[ii] != null)
            {
                double alpha_i = (u - knots[i]) / (knots[i + ord] - knots[i]);
                var P_check_homo = alpha_i * temp[ii].HomogeneousPosition + (1.0 - alpha_i) * temp[ii - 1].HomogeneousPosition;
                var P_check = new ControlPoint(P_check_homo.X / P_check_homo.W, P_check_homo.Y / P_check_homo.W, P_check_homo.Z / P_check_homo.W, P_check_homo.W);
                
                double dist = controlPoints[i].Position.DistanceTo(P_check.Position);
                if (dist > tolerance)
                    return (false, knots, controlPoints);
            }

            // Removal succeeded, create new control points array
            ControlPoint[] newControlPoints = new ControlPoint[n];

            // Copy unchanged control points at the beginning
            for (int k = 0; k < first; k++)
            {
                newControlPoints[k] = controlPoints[k];
            }

            // Copy updated control points from temp
            i = first;
            j = last;
            while (j - i > 0)
            {
                newControlPoints[i] = temp[i - off];
                newControlPoints[j] = temp[j - off];
                i++;
                j--;
            }

            // Shift remaining control points
            int fout = (2 * r - s - p) / 2; // first control point out
            for (int k = last + 1; k <= n; k++)
            {
                newControlPoints[k - 1] = controlPoints[k];
            }

            // Remove knot from knot vector
            double[] newKnots = new double[m];
            for (int k = 0; k < r; k++)
            {
                newKnots[k] = knots[k];
            }
            for (int k = r + 1; k <= m; k++)
            {
                newKnots[k - 1] = knots[k];
            }

            return (true, newKnots, newControlPoints);
        }

        /// <summary>
        /// (en) Removes a knot from a NURBS curve while preserving the curve shape within a tolerance.
        /// (ja) 許容誤差内で曲線の形状を保ちながら、NURBS曲線からノットを削除します
        /// </summary>
        /// <param name="curve">Target NURBS curve</param>
        /// <param name="u">Knot value to remove</param>
        /// <param name="tolerance">Maximum allowed deviation (default: 1e-6)</param>
        /// <returns>Tuple of (success, new curve). Success is false if removal would exceed tolerance.</returns>
        public static (bool success, NurbsCurve? curve) RemoveKnot(NurbsCurve curve, double u, double tolerance = 1e-6)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            var (success, newKnots, newControlPoints) = RemoveKnot(
                curve.Degree,
                curve.KnotVector.Knots,
                curve.ControlPoints,
                u,
                tolerance);

            if (!success)
                return (false, null);

            var newCurve = new NurbsCurve(curve.Degree, new KnotVector(newKnots, curve.Degree), newControlPoints);
            return (true, newCurve);
        }

        /// <summary>
        /// (en) Removes a knot from a NURBS surface in the specified direction while preserving the shape within a tolerance.
        /// (ja) 許容誤差内で形状を保ちながら、指定方向でNURBSサーフェスからノットを削除します
        /// </summary>
        /// <param name="surface">Target NURBS surface</param>
        /// <param name="u">Knot value to remove</param>
        /// <param name="direction">Direction (U or V) in which to remove the knot</param>
        /// <param name="tolerance">Maximum allowed deviation (default: 1e-6)</param>
        /// <returns>Tuple of (success, new surface). Success is false if removal would exceed tolerance.</returns>
        public static (bool success, NurbsSurface? surface) RemoveKnot(NurbsSurface surface, double u, SurfaceDirection direction, double tolerance = 1e-6)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            if (direction == SurfaceDirection.U)
            {
                int nV = surface.ControlPoints[0].Length;
                int oldNU = surface.ControlPoints.Length;

                List<ControlPoint[]> newColumns = new List<ControlPoint[]>();
                double[]? newKnots = null;
                bool allSuccess = true;

                for (int j = 0; j < nV; j++)
                {
                    ControlPoint[] columnCPs = new ControlPoint[oldNU];
                    for (int i = 0; i < oldNU; i++)
                    {
                        columnCPs[i] = surface.ControlPoints[i][j];
                    }

                    var (success, colNewKnots, colNewControlPoints) = RemoveKnot(
                        surface.DegreeU,
                        surface.KnotVectorU.Knots,
                        columnCPs,
                        u,
                        tolerance);

                    if (!success)
                    {
                        allSuccess = false;
                        break;
                    }

                    newKnots = colNewKnots;
                    newColumns.Add(colNewControlPoints);
                }

                if (!allSuccess || newKnots == null)
                    return (false, null);

                int newNU = newColumns[0].Length;
                ControlPoint[][] newCPs = new ControlPoint[newNU][];
                for (int i = 0; i < newNU; i++)
                {
                    newCPs[i] = new ControlPoint[nV];
                    for (int j = 0; j < nV; j++)
                    {
                        newCPs[i][j] = newColumns[j][i];
                    }
                }

                var newSurface = new NurbsSurface(
                    surface.DegreeU,
                    surface.DegreeV,
                    new KnotVector(newKnots, surface.DegreeU),
                    surface.KnotVectorV,
                    newCPs);

                return (true, newSurface);
            }
            else // V direction
            {
                int nU = surface.ControlPoints.Length;
                int oldNV = surface.ControlPoints[0].Length;

                ControlPoint[][] newCPs = new ControlPoint[nU][];
                double[]? newKnots = null;
                bool allSuccess = true;

                for (int i = 0; i < nU; i++)
                {
                    ControlPoint[] rowCPs = surface.ControlPoints[i];

                    var (success, rowNewKnots, rowNewControlPoints) = RemoveKnot(
                        surface.DegreeV,
                        surface.KnotVectorV.Knots,
                        rowCPs,
                        u,
                        tolerance);

                    if (!success)
                    {
                        allSuccess = false;
                        break;
                    }

                    newKnots = rowNewKnots;
                    newCPs[i] = rowNewControlPoints;
                }

                if (!allSuccess || newKnots == null)
                    return (false, null);

                var newSurface = new NurbsSurface(
                    surface.DegreeU,
                    surface.DegreeV,
                    surface.KnotVectorU,
                    new KnotVector(newKnots, surface.DegreeV),
                    newCPs);

                return (true, newSurface);
            }
        }

        /// <summary>
        /// (en) Refines a knot vector by inserting a given set of knots (duplicates allowed) without changing the curve shape.
        /// (ja) 与えられたノット集合（重複可）を形状を変えずに挿入しノットベクトルを細分化します。
        /// knot [0 0 0 1 1 1] , degree 2, given refine [0.25,0.25,0.25,0.5,0.75] => [0 0 0 0 0.25 0.25 0.5 0.75 1 1 1 1]
        /// </summary>
        /// <param name="degree">Curve degree</param>
        /// <param name="knots">Original knot vector</param>
        /// <param name="controlPoints">Original control points</param>
        /// <param name="refineKnots">Knots to insert (may contain duplicates, unsorted)</param>
        /// <returns>New knot vector and control points</returns>
        public static (double[] newKnots, ControlPoint[] newControlPoints) RefineKnot(int degree, double[] knots, ControlPoint[] controlPoints, double[] refineKnots)
        {
            Guard.ThrowIfNull(knots, nameof(knots));
            Guard.ThrowIfNull(controlPoints, nameof(controlPoints));
            Guard.ThrowIfNull(refineKnots, nameof(refineKnots));
            if (degree < 1)
                throw new ArgumentException("Degree must be at least 1.", nameof(degree));
            if (refineKnots.Length == 0)
                return (knots, controlPoints);

            double start = knots[0];
            double end = knots[^1];

            // Grouping: Calculate desired insertion count for each value
            var grouped = refineKnots
                .Where(k => k >= start && k <= end) // ignore out-of-range knots
                .OrderBy(k => k)
                .GroupBy(k => k)
                .Select(g => (value: g.Key, count: g.Count()))
                .ToList();

            double[] currentKnots = (double[])knots.Clone();
            ControlPoint[] currentCP = (ControlPoint[])controlPoints.Clone();

            foreach (var (value, requestedCount) in grouped)
            {
                // Endpoints (clamped) usually do not need/cannot be added: multiplicity is degree+1, so skip
                bool isEndpoint = LinAlg.ApproxEqual(value, start) || LinAlg.ApproxEqual(value, end);
                int existingMultiplicity = GetMultiplicity(currentKnots, value);
                int maxMultiplicity = isEndpoint ? degree + 1 : degree; // internal: <= degree, end: <= degree+1
                int possibleAdds = Math.Max(0, maxMultiplicity - existingMultiplicity);
                int times = Math.Min(possibleAdds, requestedCount);
                if (times <= 0) continue; // Skip if no insertion is possible/needed

                var (newKnots, newControlPoints) = InsertKnot(degree, currentKnots, currentCP, value, times);
                currentKnots = newKnots;
                currentCP = newControlPoints;
            }

            return (currentKnots, currentCP);
        }

        /// <summary>
        /// (en) Wrapper to refine a NurbsCurve with given knots.
        /// (ja) 与えられたノット集合で NURBS 曲線を細分化するラッパー。
        /// </summary>
        /// <param name="curve">Target curve</param>
        /// <param name="refineKnots">Knots to insert (may contain duplicates)</param>
        /// <returns>Refined curve</returns>
        public static NurbsCurve RefineKnot(NurbsCurve curve, double[] refineKnots)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            Guard.ThrowIfNull(refineKnots, nameof(refineKnots));
            var (newKnots, newControlPoints) = RefineKnot(curve.Degree, curve.KnotVector.Knots, curve.ControlPoints, refineKnots);
            return new NurbsCurve(curve.Degree, new KnotVector(newKnots, curve.Degree), newControlPoints);
        }

        /// <summary>
        /// (en) Refines a NURBS surface by inserting a given set of knots in the specified direction.
        /// (ja) 与えられたノット集合を指定方向に挿入し、NURBSサーフェスを細分化します。
        /// </summary>
        /// <param name="surface">Target NURBS surface</param>
        /// <param name="refineKnots">Knots to insert (may contain duplicates)</param>
        /// <param name="direction">Direction (U or V) in which to refine the knots</param>
        /// <returns>Refined NURBS surface</returns>
        public static NurbsSurface RefineKnot(NurbsSurface surface, double[] refineKnots, SurfaceDirection direction)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            Guard.ThrowIfNull(refineKnots, nameof(refineKnots));
            
            if (direction == SurfaceDirection.U)
            {
                // Refine knots in U direction: treat each V-column as an independent curve
                int nV = surface.ControlPoints[0].Length;
                int oldNU = surface.ControlPoints.Length;

                // Extract control points for each V column and perform knot refinement
                List<ControlPoint[]> newColumns = new List<ControlPoint[]>();
                double[]? newKnots = null;

                for (int j = 0; j < nV; j++)
                {
                    // Extract control points for this V column (varying U)
                    ControlPoint[] columnCPs = new ControlPoint[oldNU];
                    for (int i = 0; i < oldNU; i++)
                    {
                        columnCPs[i] = surface.ControlPoints[i][j];
                    }

                    // Refine knots for this column
                    var (colNewKnots, colNewControlPoints) = RefineKnot(surface.DegreeU, surface.KnotVectorU.Knots, columnCPs, refineKnots);
                    newKnots = colNewKnots; // Same for all columns
                    newColumns.Add(colNewControlPoints);
                }

                // Rebuild control points 2D array [U][V]
                int newNU = newColumns[0].Length;
                ControlPoint[][] newCPs = new ControlPoint[newNU][];
                for (int i = 0; i < newNU; i++)
                {
                    newCPs[i] = new ControlPoint[nV];
                    for (int j = 0; j < nV; j++)
                    {
                        newCPs[i][j] = newColumns[j][i];
                    }
                }

                if (newKnots == null)
                    throw new InvalidOperationException("Failed to refine knots: no columns processed.");

                return new NurbsSurface(surface.DegreeU, surface.DegreeV, new KnotVector(newKnots, surface.DegreeU), surface.KnotVectorV, newCPs);
            }
            else // V direction
            {
                // Refine knots in V direction: treat each U-row as an independent curve
                int nU = surface.ControlPoints.Length;
                int oldNV = surface.ControlPoints[0].Length;

                // Extract control points for each U row and perform knot refinement
                ControlPoint[][] newCPs = new ControlPoint[nU][];
                double[]? newKnots = null;

                for (int i = 0; i < nU; i++)
                {
                    // Extract control points for this U row (varying V)
                    ControlPoint[] rowCPs = surface.ControlPoints[i]; // Already an array in V direction

                    // Refine knots for this row
                    var (rowNewKnots, rowNewControlPoints) = RefineKnot(surface.DegreeV, surface.KnotVectorV.Knots, rowCPs, refineKnots);
                    newKnots = rowNewKnots; // Same for all rows
                    newCPs[i] = rowNewControlPoints;
                }

                if (newKnots == null)
                    throw new InvalidOperationException("Failed to refine knots: no rows processed.");

                return new NurbsSurface(surface.DegreeU, surface.DegreeV, surface.KnotVectorU, new KnotVector(newKnots, surface.DegreeV), newCPs);
            }
        }
        private static int GetMultiplicity(double[] knots, double value)
        {
            int m = 0;
            for (int i = 0; i < knots.Length; i++)
            {
                if (LinAlg.ApproxEqual(knots[i], value)) m++;
            }
            return m;
        }
    }
}
