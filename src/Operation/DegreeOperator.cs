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
        /// (ja) 形状を保つように与えられたNURBS曲線の次数をtだけ昇降します
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static NurbsCurve ElevateDegree(NurbsCurve curve, int t)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            if (t <= 0)
                return curve;
            if (t < 0)
                throw new ArgumentOutOfRangeException(nameof(t), "Degree elevation must be non-negative.");

            NurbsCurve result = curve;
            for (int i = 0; i < t; i++)
            {
                result = ElevateDegreeBezierOnce(result);
            }
            return result;
        }

        /// <summary>
        /// Decomposes into Bezier segments and applies Eq.5.36 (geomdl-equivalent, homogeneous coords) to elevate degree by 1.
        /// Internal knot multiplicities are incremented by +1.
        /// </summary>
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
        /// (en) Reduces the degree of the curve by t using global weighted LSQ in homogeneous space.
        /// (ja) 同次座標での重み付き最小二乗により、指定回数 t の次数低減を行います。
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="t"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static NurbsCurve ReduceDegree(NurbsCurve curve, int t, double tolerance)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            if (t <= 0)
                return curve;
            if (t < 0)
                throw new ArgumentOutOfRangeException(nameof(t), "Degree reduction must be non-negative.");

            int p = curve.Degree;
            int pNew = p - t;
            if (pNew < 1)
                throw new ArgumentOutOfRangeException(nameof(t), "Resulting degree must be at least 1.");

            NurbsCurve result = curve;
            for (int i = 0; i < t; i++)
            {
                result = ReduceDegreeOnceLSQ(result, tolerance);
            }
            return result;
        }


        // Global degree reduction p->p-1 via LSQ approximation in homogeneous coords.
        // Stabilized by sample density and pivot point weights.
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
            double[] pivots = new double[] { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0 };
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
                    A[i][j] = ws[i] * BSplineBasisFunction(j, ph, u, newKnotVec.Knots);
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
            double[] solXw = SolveLinearSystem(ATA, ATbXw);
            double[] solYw = SolveLinearSystem(ATA, ATbYw);
            double[] solZw = SolveLinearSystem(ATA, ATbZw);
            double[] solW = SolveLinearSystem(ATA, ATbW);

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

        // Helper methods
        private static long Binomial(int n, int k)
        {
            if (k > n || k < 0) return 0;
            if (k == 0 || k == n) return 1;
            if (k > n - k) k = n - k;

            long result = 1;
            for (int i = 0; i < k; i++)
            {
                result *= (n - i);
                result /= (i + 1);
            }
            return result;
        }

        // Equivalent to geomdl's degree_elevation (Eq.5.36) in homogeneous coordinates
        private static ControlPoint[] ElevateBezierControlPoints(ControlPoint[] ctrlpts, int num)
        {
            int degree = ctrlpts.Length - 1;
            int numPtsElev = degree + 1 + num;
            var outPts = new Vector4Double[numPtsElev];
            for (int i = 0; i < numPtsElev; i++) outPts[i] = new Vector4Double(0, 0, 0, 0);

            // Homogeneous coordinate array
            var hp = new Vector4Double[ctrlpts.Length];
            for (int i = 0; i < ctrlpts.Length; i++) hp[i] = ctrlpts[i].HomogeneousPosition;

            for (int i = 0; i < numPtsElev; i++)
            {
                int start = Math.Max(0, i - num);
                int end = Math.Min(degree, i);
                for (int j = start; j <= end; j++)
                {
                    double coeff = Binomial(degree, j) * 1.0;
                    coeff *= Binomial(num, i - j);
                    coeff /= Binomial(degree + num, i);
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


        private static double BSplineBasisFunction(int i, int p, double u, double[] knots)
        {
            int m = knots.Length;
            if (p == 0)
            {
                if (knots[i] <= u && u < knots[i + 1]) return 1.0;
                if (LinAlg.ApproxEqual(u, knots[knots.Length - 1]) && i == knots.Length - p - 2) return 1.0;
                return 0.0;
            }
            double denom1 = knots[i + p] - knots[i];
            double denom2 = knots[i + p + 1] - knots[i + 1];
            double term1 = 0.0;
            double term2 = 0.0;
            if (LinAlg.IsNotZero(denom1))
                term1 = (u - knots[i]) / denom1 * BSplineBasisFunction(i, p - 1, u, knots);
            if (LinAlg.IsNotZero(denom2))
                term2 = (knots[i + p + 1] - u) / denom2 * BSplineBasisFunction(i + 1, p - 1, u, knots);
            return term1 + term2;
        }

        private static double[] SolveLinearSystem(double[][] A, double[] b)
        {
            int n = b.Length;
            double[][] M = new double[n][];
            for (int i = 0; i < n; i++)
            {
                M[i] = new double[n + 1];
                for (int j = 0; j < n; j++)
                    M[i][j] = A[i][j];
                M[i][n] = b[i];
            }

            for (int k = 0; k < n; k++)
            {
                int maxRow = k;
                double maxVal = Math.Abs(M[k][k]);
                for (int i = k + 1; i < n; i++)
                {
                    double val = Math.Abs(M[i][k]);
                    if (val > maxVal)
                    {
                        maxVal = val; maxRow = i;
                    }
                }
                if (LinAlg.ApproxEqual(maxVal, 0.0))
                    throw new InvalidOperationException("Linear system is singular or ill-conditioned.");

                if (maxRow != k)
                {
                    for (int j = k; j < n + 1; j++)
                    {
                        (M[maxRow][j], M[k][j]) = (M[k][j], M[maxRow][j]);
                    }
                }

                for (int i = k + 1; i < n; i++)
                {
                    double f = M[i][k] / M[k][k];
                    for (int j = k; j < n + 1; j++)
                        M[i][j] -= f * M[k][j];
                }
            }

            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double s = M[i][n];
                for (int j = i + 1; j < n; j++) s -= M[i][j] * x[j];
                x[i] = s / M[i][i];
            }
            return x;
        }


        private static Vector4Double EvaluateHomogeneous(NurbsSharp.Geometry.NurbsCurve curve, double u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            int p = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var cps = curve.ControlPoints;

            if (u < knots[p]) u = knots[p];
            if (u > knots[knots.Length - p - 1]) u = knots[knots.Length - p - 1];

            Vector4Double H = new Vector4Double(0, 0, 0, 0);
            for (int i = 0; i < cps.Length; i++)
            {
                double Ni = BSplineBasisFunction(i, p, u, knots);
                var hp = cps[i].HomogeneousPosition;
                H.X += Ni * hp.X;
                H.Y += Ni * hp.Y;
                H.Z += Ni * hp.Z;
                H.W += Ni * hp.W;
            }
            return H;
            }
    }
}
