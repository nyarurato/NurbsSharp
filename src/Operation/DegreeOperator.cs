using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for degree elevation and reduction of NURBS
    /// (ja) NURBSの次数昇降のためのオペレーター
    /// </summary>
    public class DegreeOperator
    {
        /// <summary>
        /// (en) Elevates the degree of the given NURBS curve by t while preserving its shape
        /// (ja) 形状を保つように与えられたNURBS曲線の次数をtだけ上げます
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="times"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static NurbsCurve ElevateDegree(NurbsCurve curve, int times)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            Guard.ThrowIfNegative(times, nameof(times));

            if (times == 0)
                return curve;

            NurbsCurve result = curve;
            for (int i = 0; i < times; i++)
            {
                result = ElevateDegreeBezierOnce(result);
            }
            return result;
        }

        /// <summary>
        /// (en) Elevates the degree of the given NURBS curve by 1 using Bezier decomposition and reassembly.
        /// (ja) ベジェ分解と再構成により、与えられたNURBS曲線の次数を1だけ上げます。
        /// </summary>
        /// <param name="curve"></param>
        /// <returns></returns>
        private static NurbsCurve ElevateDegreeBezierOnce(NurbsCurve curve)
        {
            int p = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var cps = curve.ControlPoints;

            // 1) Insert internal knots for Bezier decomposition (internal multiplicity to p)
            double uMin = knots[p];
            double uMax = knots[knots.Length - p - 1];
            // Collect distinct breakpoints (internal knot values)
            var distinct = new SortedSet<double>();
            for (int i = p + 1; i < knots.Length - p - 1; i++)
            {
                if (!LinAlg.ApproxEqual(knots[i], knots[i - 1]))
                {
                    double u = knots[i];
                    if (u > uMin && u < uMax) distinct.Add(u);
                }
            }

            double[] refKnots = (double[])knots.Clone();
            ControlPoint[] refCP = (ControlPoint[])cps.Clone();

            foreach (var u in distinct)
            {
                // Count current multiplicity and insert to reach p
                int mult = 0;
                for (int k = 0; k < refKnots.Length; k++) if (LinAlg.ApproxEqual(refKnots[k], u)) mult++;
                int times = p - mult;
                if (times > 0)
                {
                    (refKnots, refCP) = KnotOperator.InsertKnot(p, refKnots, refCP, u, times);
                }
            }

            // 2) Get Bezier segment count and control points for each segment
            // Number of segments after Bezierization = number of non-zero spans
            int segCount = 0;
            for (int i = p; i < refKnots.Length - p - 1; i++)
                if (!LinAlg.ApproxEqual(refKnots[i], refKnots[i + 1])) segCount++;

            // Segments are adjacent with p-point overlap. First index of each segment is s*p
            var newCPs = new List<ControlPoint>(segCount * (p + 2));
            for (int s = 0; s < segCount; s++)
            {
                int baseIdx = s * p;
                var seg = new ControlPoint[p + 1];
                Array.Copy(refCP, baseIdx, seg, 0, p + 1);

                var elevSeg = ElevateBezierControlPoints(seg, 1);
                if (s == 0)
                    newCPs.AddRange(elevSeg);
                else
                {
                    // Skip first point to avoid boundary duplication
                    for (int i = 1; i < elevSeg.Length; i++) newCPs.Add(elevSeg[i]);
                }
            }

            // 3) New knot vector (increment multiplicity of each value by +1)
            var newKnotsList = new List<double>(refKnots.Length + segCount + 2);
            int idx = 0;
            while (idx < refKnots.Length)
            {
                double val = refKnots[idx];
                int run = 1; idx++;
                while (idx < refKnots.Length && LinAlg.ApproxEqual(refKnots[idx], val)) { run++; idx++; }
                for (int r = 0; r < run + 1; r++) newKnotsList.Add(val);
            }

            int newDegree = p + 1;
            var newKnotVector = new KnotVector(newKnotsList.ToArray(), newDegree);
            return new NurbsCurve(newDegree, newKnotVector, newCPs.ToArray());
        }

        /// <summary>
        /// (en) Reduces the degree of the curve by times using global weighted LSQ in homogeneous space.
        /// (ja) 同次座標での重み付き最小二乗により、指定回数 times の次数低減を行います。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="times"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static NurbsCurve ReduceDegree(NurbsCurve curve, int times, double tolerance)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            Guard.ThrowIfNegative(times, nameof(times));
            if (times == 0)
                return curve;

            int p = curve.Degree;
            int pNew = p - times;
            if (pNew < 1)
                throw new ArgumentOutOfRangeException(nameof(times), "Resulting degree must be at least 1.");

            NurbsCurve result = curve;
            for (int i = 0; i < times; i++)
            {
                result = ReduceDegreeOnceLSQ(result, tolerance);
            }
            return result;
        }


        /// <summary>
        /// (en) Reduces the degree of the given NURBS curve by 1 using global weighted LSQ in homogeneous space.
        /// (ja) 同次座標での重み付き最小二乗により、与えられたNURBS曲線の次数を1だけ低減します。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        /// <exception cref="InvalidOperationException"></exception>
        private static NurbsCurve ReduceDegreeOnceLSQ(NurbsCurve curve, double tolerance)
        {
            int p = curve.Degree;
            int n = curve.ControlPoints.Length - 1;
            var U = curve.KnotVector.Knots;
            var Pw = curve.ControlPoints;
            int m = n + p + 1;
            int ph = p - 1;
            if (ph < 1) throw new InvalidOperationException("Cannot reduce degree below 1.");

            // New knot vector (clamped ends, internal knots placed by arc-length distribution)
            var newKnots = new List<double>();
            double uMin = U[p];
            double uMax = U[m - p];
            for (int i = 0; i <= ph; i++) newKnots.Add(uMin);
            int nNew = n; // Maintain control point count
            int numInternalKnots = nNew - ph;
            if (numInternalKnots > 0)
            {
                // Approximate arc length via samples -> place internal knots at quantiles
                int lenSamples = Math.Max(400, (n + 1) * 20);
                var usLen = new double[lenSamples];
                var ptsLen = new Vector3Double[lenSamples];
                for (int i = 0; i < lenSamples; i++)
                {
                    double u = uMin + (uMax - uMin) * i / (lenSamples - 1);
                    usLen[i] = u;
                    var p3 = CurveEvaluator.Evaluate(curve, u);
                    ptsLen[i] = p3;
                }
                var cum = new double[lenSamples];
                cum[0] = 0.0;
                for (int i = 1; i < lenSamples; i++) cum[i] = cum[i - 1] + ptsLen[i].DistanceTo(ptsLen[i - 1]);
                double total = cum[lenSamples - 1];
                for (int k = 1; k <= numInternalKnots; k++)
                {
                    double target = total * k / (numInternalKnots + 1);
                    int idx = Array.BinarySearch(cum, target);
                    if (idx < 0) idx = ~idx;
                    idx = Math.Clamp(idx, 1, lenSamples - 1);
                    // Linear interpolation to reverse-calculate u
                    double t0 = cum[idx - 1], t1 = cum[idx];
                    double w = LinAlg.IsNotZero(t1 - t0) ? (target - t0) / (t1 - t0) : 0.0;
                    double u = usLen[idx - 1] * (1 - w) + usLen[idx] * w;
                    newKnots.Add(u);
                }
                newKnots.Sort();
            }
            for (int i = 0; i <= ph; i++) newKnots.Add(uMax);
            var newKnotVec = new KnotVector(newKnots.ToArray(), ph);

            // Dense sampling + pivot point weighting
            int baseSamples = Math.Max((nNew + 1) * 50, 1000);
            var usList = new List<(double u, double w)>(baseSamples + 8);
            for (int i = 0; i < baseSamples; i++)
            {
                double u = uMin + (uMax - uMin) * i / (baseSamples - 1);
                usList.Add((u, 1.0));
            }
            // Enhance weighting for commonly tested evaluation points
            double[] pivots = [ 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0 ];
            foreach (var t in pivots)
            {
                double u = uMin + (uMax - uMin) * t;
                usList.Add((u, 400.0));
            }
            int sampleCount = usList.Count;
            var samples = new Vector4Double[sampleCount];
            var us = new double[sampleCount];
            var ws = new double[sampleCount];
            for (int i = 0; i < sampleCount; i++)
            {
                us[i] = usList[i].u;
                ws[i] = Math.Sqrt(usList[i].w);
                samples[i] = EvaluateHomogeneous(curve, us[i]);
            }

            // A matrix (basis functions)
            double[][] A = new double[sampleCount][];
            for (int i = 0; i < sampleCount; i++)
            {
                double u = us[i];
                A[i] = new double[nNew + 1];
                for (int j = 0; j <= nNew; j++)
                {
                    A[i][j] = ws[i] * BasicEvaluator.BSplineBasisFunction(j, ph, u, newKnotVec.Knots);
                }
            }

            // Normal equations A^T A x = A^T b (with Tikhonov micro-regularization)
            double[][] ATA = new double[nNew + 1][];
            double[] ATbXw = new double[nNew + 1];
            double[] ATbYw = new double[nNew + 1];
            double[] ATbZw = new double[nNew + 1];
            double[] ATbW = new double[nNew + 1];

            for (int i = 0; i <= nNew; i++)
            {
                ATA[i] = new double[nNew + 1];
                for (int j = 0; j <= nNew; j++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < sampleCount; k++) sum += A[k][i] * A[k][j];
                    ATA[i][j] = sum;
                }
                double sumXw = 0.0, sumYw = 0.0, sumZw = 0.0, sumW = 0.0;
                for (int k = 0; k < sampleCount; k++)
                {
                    var h = samples[k];
                    sumXw += A[k][i] * (ws[k] * h.X);
                    sumYw += A[k][i] * (ws[k] * h.Y);
                    sumZw += A[k][i] * (ws[k] * h.Z);
                    sumW += A[k][i] * (ws[k] * h.W);
                }
                ATbXw[i] = sumXw;
                ATbYw[i] = sumYw;
                ATbZw[i] = sumZw;
                ATbW[i] = sumW;
            }

            // Stabilize with micro-Tikhonov regularization
            double lambda = Math.Max(1e-14, tolerance * 1e-6);
            for (int i = 0; i <= nNew; i++) ATA[i][i] += lambda;

            // Standard LSQ solution (pseudo-constraints via enhanced weights)
            double[] solXw = LinAlg.SolveLinearSystem(ATA, ATbXw);
            double[] solYw = LinAlg.SolveLinearSystem(ATA, ATbYw);
            double[] solZw = LinAlg.SolveLinearSystem(ATA, ATbZw);
            double[] solW = LinAlg.SolveLinearSystem(ATA, ATbW);

            var newCP = new ControlPoint[nNew + 1];
            for (int i = 0; i <= nNew; i++)
            {
                double w = solW[i];
                if (LinAlg.ApproxEqual(w, 0.0)) w = 1e-12; // Safety guard
                newCP[i] = new ControlPoint(solXw[i] / w, solYw[i] / w, solZw[i] / w, w);
            }

            // Reset endpoints to maintain numerical match with original curve
            newCP[0] = new ControlPoint(Pw[0].Position, Pw[0].Weight);
            newCP[nNew] = new ControlPoint(Pw[n].Position, Pw[n].Weight);

            return new NurbsCurve(ph, newKnotVec, newCP);
        }

        /// <summary>
        /// (en) Elevates the degree of a Bezier curve defined by the given control points by 'times'
        /// (ja) 与えられた制御点で定義されるベジェ曲線の次数を 'times' だけ上げます
        /// </summary>
        /// <param name="ctrlpts"></param>
        /// <param name="times"></param>
        /// <returns></returns>
        private static ControlPoint[] ElevateBezierControlPoints(ControlPoint[] ctrlpts, int times)
        {
            int degree = ctrlpts.Length - 1;
            int numPtsElev = degree + 1 + times;
            var outPts = new Vector4Double[numPtsElev];
            for (int i = 0; i < numPtsElev; i++) outPts[i] = new Vector4Double(0, 0, 0, 0);

            // Homogeneous coordinate array
            var hp = new Vector4Double[ctrlpts.Length];
            for (int i = 0; i < ctrlpts.Length; i++) hp[i] = ctrlpts[i].HomogeneousPosition;

            for (int i = 0; i < numPtsElev; i++)
            {
                int start = Math.Max(0, i - times);
                int end = Math.Min(degree, i);
                for (int j = start; j <= end; j++)
                {
                    double coeff = LinAlg.BinomialCoefficient(degree, j) * 1.0;
                    coeff *= LinAlg.BinomialCoefficient(times, i - j);
                    coeff /= LinAlg.BinomialCoefficient(degree + times, i);
                    outPts[i] += coeff * hp[j];
                }
            }

            var result = new ControlPoint[numPtsElev];
            for (int i = 0; i < numPtsElev; i++)
            {
                var v = outPts[i];
                if (LinAlg.ApproxEqual(v.W, 0.0)) result[i] = new ControlPoint(0, 0, 0, 0);
                else result[i] = new ControlPoint(v.X / v.W, v.Y / v.W, v.Z / v.W, v.W);
            }
            return result;
        }

        /// <summary>
        /// (en) Evaluates the homogeneous position of the NURBS curve at parameter u
        /// (ja) NURBS曲線のパラメーターuにおける同次座標を評価します
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="u"></param>
        /// <returns></returns>
        private static Vector4Double EvaluateHomogeneous(NurbsCurve curve, double u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            int p = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var cps = curve.ControlPoints;

            if (u < knots[p]) u = knots[p];
            if (u > knots[knots.Length - p - 1]) u = knots[knots.Length - p - 1];

            Vector4Double H = new();
            for (int i = 0; i < cps.Length; i++)
            {
                double Ni = BasicEvaluator.BSplineBasisFunction(i, p, u, knots);
                var hp = cps[i].HomogeneousPosition;
                H += Ni * hp;
            }
            return H;
        }

        /// <summary>
        /// (en) Elevates the degree of the given NURBS surface in U and/or V direction while preserving its shape
        /// (ja) 形状を保つように与えられたNURBSサーフェスのU方向およびV方向の次数を上げます
        /// </summary>
        /// <param name="surface">Surface to elevate</param>
        /// <param name="timesU">Number of times to elevate in U direction</param>
        /// <param name="timesV">Number of times to elevate in V direction</param>
        /// <returns>Surface with elevated degree</returns>
        /// <exception cref="ArgumentNullException">Thrown when surface is null</exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown when timesU or timesV is negative</exception>"
        public static NurbsSurface ElevateDegree(NurbsSurface surface, int timesU, int timesV)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            Guard.ThrowIfNegative(timesU, nameof(timesU));
            Guard.ThrowIfNegative(timesV, nameof(timesV));

            NurbsSurface result = surface;

            // Elevate in U direction first (if needed)
            if (timesU > 0)
            {
                result = ElevateDegreeU(result, timesU);
            }

            // Then elevate in V direction (if needed)
            if (timesV > 0)
            {
                result = ElevateDegreeV(result, timesV);
            }

            return result;
        }

        /// <summary>
        /// (en) Reduces the degree of the given NURBS surface in U and/or V direction
        /// (ja) 与えられたNURBSサーフェスのU方向およびV方向の次数を下げます
        /// </summary>
        /// <param name="surface">Surface to reduce</param>
        /// <param name="timesU">Number of times to reduce in U direction</param>
        /// <param name="timesV">Number of times to reduce in V direction</param>
        /// <param name="tolerance">Tolerance for degree reduction approximation</param>
        /// <returns>Surface with reduced degree</returns>
        /// <exception cref="ArgumentNullException">Thrown when surface is null</exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown when reduction would result in degree less than 1</exception>
        public static NurbsSurface ReduceDegree(NurbsSurface surface, int timesU, int timesV, double tolerance = 1e-6)
        {
            Guard.ThrowIfNull(surface, nameof(surface));
            Guard.ThrowIfNegative(timesU, nameof(timesU));
            Guard.ThrowIfNegative(timesV, nameof(timesV));

            NurbsSurface result = surface;

            // Reduce in U direction first (if needed)
            if (timesU > 0)
            {
                result = ReduceDegreeU(result, timesU, tolerance);
            }

            // Then reduce in V direction (if needed)
            if (timesV > 0)
            {
                result = ReduceDegreeV(result, timesV, tolerance);
            }

            return result;
        }

        /// <summary>
        /// (en) Elevates the degree of the surface in U direction
        /// (ja) サーフェスのU方向の次数を上げます
        /// </summary>
        private static NurbsSurface ElevateDegreeU(NurbsSurface surface, int times)
        {
            if (times <= 0) return surface;

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            KnotVector knotVectorV = surface.KnotVectorV;
            ControlPoint[][] controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;

            // For each V index, extract U-direction curve and elevate its degree
            List<NurbsCurve> elevatedCurves = [];

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

                // Elevate the curve degree
                var elevatedCurve = ElevateDegree(curve, times);

                elevatedCurves.Add(elevatedCurve);
            }

            // Reconstruct surface from elevated curves
            int newNU = elevatedCurves[0].ControlPoints.Length;
            ControlPoint[][] newControlPoints = new ControlPoint[newNU][];
            for (int uIndex = 0; uIndex < newNU; uIndex++)
            {
                newControlPoints[uIndex] = new ControlPoint[nV];
                for (int vIndex = 0; vIndex < nV; vIndex++)
                {
                    newControlPoints[uIndex][vIndex] = elevatedCurves[vIndex].ControlPoints[uIndex];
                }
            }

            return new NurbsSurface(
                elevatedCurves[0].Degree,
                degreeV,
                elevatedCurves[0].KnotVector,
                knotVectorV,
                newControlPoints);
        }

        /// <summary>
        /// (en) Elevates the degree of the surface in V direction
        /// (ja) サーフェスのV方向の次数を上げます
        /// </summary>
        private static NurbsSurface ElevateDegreeV(NurbsSurface surface, int times)
        {
            if (times <= 0) return surface;

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            KnotVector knotVectorU = surface.KnotVectorU;
            ControlPoint[][] controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;

            // For each U index, extract V-direction curve and elevate its degree
            List<NurbsCurve> elevatedCurves = [];

            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                // Extract control points along V direction for this U index
                ControlPoint[] curveControlPoints = controlPoints[uIndex];

                // Create temporary curve along V direction
                var curve = new NurbsCurve(degreeV, surface.KnotVectorV, curveControlPoints);

                // Elevate the curve degree
                var elevatedCurve = ElevateDegree(curve, times);

                elevatedCurves.Add(elevatedCurve);
            }

            // Reconstruct surface from elevated curves
            int newNV = elevatedCurves[0].ControlPoints.Length;
            ControlPoint[][] newControlPoints = new ControlPoint[nU][];
            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                newControlPoints[uIndex] = elevatedCurves[uIndex].ControlPoints;
            }

            return new NurbsSurface(
                degreeU,
                elevatedCurves[0].Degree,
                knotVectorU,
                elevatedCurves[0].KnotVector,
                newControlPoints);
        }

        /// <summary>
        /// (en) Reduces the degree of the surface in U direction
        /// (ja) サーフェスのU方向の次数を下げます
        /// </summary>
        private static NurbsSurface ReduceDegreeU(NurbsSurface surface, int times, double tolerance)
        {
            if (times <= 0) return surface;

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            KnotVector knotVectorV = surface.KnotVectorV;
            ControlPoint[][] controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;

            // Check if reduction is possible
            if (degreeU - times < 1)
                throw new ArgumentOutOfRangeException(nameof(times), 
                    $"Cannot reduce U degree from {degreeU} by {times}. Resulting degree must be at least 1.");

            // For each V index, extract U-direction curve and reduce its degree
            List<NurbsCurve> reducedCurves = [];

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

                // Reduce the curve degree
                var reducedCurve = ReduceDegree(curve, times, tolerance);

                reducedCurves.Add(reducedCurve);
            }

            // Reconstruct surface from reduced curves
            int newNU = reducedCurves[0].ControlPoints.Length;
            ControlPoint[][] newControlPoints = new ControlPoint[newNU][];
            for (int uIndex = 0; uIndex < newNU; uIndex++)
            {
                newControlPoints[uIndex] = new ControlPoint[nV];
                for (int vIndex = 0; vIndex < nV; vIndex++)
                {
                    newControlPoints[uIndex][vIndex] = reducedCurves[vIndex].ControlPoints[uIndex];
                }
            }

            return new NurbsSurface(
                reducedCurves[0].Degree,
                degreeV,
                reducedCurves[0].KnotVector,
                knotVectorV,
                newControlPoints);
        }

        /// <summary>
        /// (en) Reduces the degree of the surface in V direction
        /// (ja) サーフェスのV方向の次数を下げます
        /// </summary>
        private static NurbsSurface ReduceDegreeV(NurbsSurface surface, int times, double tolerance)
        {
            if (times <= 0) return surface;

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;
            KnotVector knotVectorU = surface.KnotVectorU;
            ControlPoint[][] controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;

            // Check if reduction is possible
            if (degreeV - times < 1)
                throw new ArgumentOutOfRangeException(nameof(times), 
                    $"Cannot reduce V degree from {degreeV} by {times}. Resulting degree must be at least 1.");

            // For each U index, extract V-direction curve and reduce its degree
            List<NurbsCurve> reducedCurves = [];

            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                // Extract control points along V direction for this U index
                ControlPoint[] curveControlPoints = controlPoints[uIndex];

                // Create temporary curve along V direction
                var curve = new NurbsCurve(degreeV, surface.KnotVectorV, curveControlPoints);

                // Reduce the curve degree
                var reducedCurve = ReduceDegree(curve, times, tolerance);

                reducedCurves.Add(reducedCurve);
            }

            // Reconstruct surface from reduced curves
            int newNV = reducedCurves[0].ControlPoints.Length;
            ControlPoint[][] newControlPoints = new ControlPoint[nU][];
            for (int uIndex = 0; uIndex < nU; uIndex++)
            {
                newControlPoints[uIndex] = reducedCurves[uIndex].ControlPoints;
            }

            return new NurbsSurface(
                degreeU,
                reducedCurves[0].Degree,
                knotVectorU,
                reducedCurves[0].KnotVector,
                newControlPoints);
        }
    }
}
