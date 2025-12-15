using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;

namespace NurbsSharp.Analysis
{
    /// <summary>
    /// (en) Arc-length parameterization helper for NURBS curves (u - s)
    /// (ja) NURBS曲線の弧長パラメータ化ヘルパ (u - s)
    /// </summary>
    public sealed class CurveArcLengthParameterization : BasicAnalyzer
    {
        private static readonly double[] ZeroArcLengthTable = new double[] { 0.0 };

        private readonly NurbsCurve _curve;
        private readonly double[] _us;
        private readonly double[] _ss;

        /// <summary>
        /// (en) Minimum parameter value for evaluation domain.
        /// (ja) 評価ドメインの最小パラメータ値。
        /// </summary>
        public double UMin { get; }

        /// <summary>
        /// (en) Maximum parameter value for evaluation domain.
        /// (ja) 評価ドメインの最大パラメータ値。
        /// </summary>
        public double UMax { get; }

        /// <summary>
        /// (en) Total arc length from UMin to UMax.
        /// (ja) UMin から UMax までの総弧長。
        /// </summary>
        public double TotalLength { get; }

        internal CurveArcLengthParameterization(NurbsCurve curve, double uMin, double uMax, double[] us, double[] ss)
        {
            _curve = curve;
            UMin = uMin;
            UMax = uMax;
            _us = us;
            _ss = ss;
            TotalLength = ss.Length == 0 ? 0.0 : ss[^1];
        }

        /// <summary>
        /// (en) Get arc length s from UMin to u.
        /// (ja) UMin から u までの弧長 s を取得します。
        /// </summary>
        public double GetArcLengthAt(double u, bool accurate = false)
        {
            if (_us.Length == 0)
                return 0.0;

            if (double.IsNaN(u) || double.IsInfinity(u))
                throw new ArgumentOutOfRangeException(nameof(u), "u must be a finite number.");

            u = Math.Clamp(u, UMin, UMax);

            int idx = FindSegmentByU(u);
            if (idx >= _us.Length - 1)
                return TotalLength;

            double u0 = _us[idx];
            double u1 = _us[idx + 1];
            double s0 = _ss[idx];
            double s1 = _ss[idx + 1];

            if (u1 <= u0)
                return s0;

            if (!accurate)
            {
                double t = (u - u0) / (u1 - u0);
                return s0 + (s1 - s0) * t;
            }

            return s0 + IntegrateArcLength(u0, u);
        }

        /// <summary>
        /// (en) Get parameter u such that arc length from UMin to u is approximately s.
        /// (ja) UMin から u までの弧長が s になるようなパラメータ u を求めます。
        /// </summary>
        public double GetParameterAtArcLength(double s, double tolerance = 1e-8, int maxIterations = 20)
        {
            if (_us.Length == 0)
                return 0.0;

            if (double.IsNaN(s) || double.IsInfinity(s))
                throw new ArgumentOutOfRangeException(nameof(s), "s must be a finite number.");

            if (tolerance <= 0)
                throw new ArgumentOutOfRangeException(nameof(tolerance), "tolerance must be positive.");

            if (maxIterations < 1)
                throw new ArgumentOutOfRangeException(nameof(maxIterations), "maxIterations must be >= 1.");

            if (s <= 0.0)
                return UMin;
            if (s >= TotalLength)
                return UMax;

            int idx = FindSegmentByS(s);
            if (idx >= _us.Length - 1)
                return UMax;

            double u0 = _us[idx];
            double u1 = _us[idx + 1];
            double s0 = _ss[idx];
            double s1 = _ss[idx + 1];

            if (s1 <= s0 || u1 <= u0)
                return u0;

            // Initial guess by linear interpolation in table
            double u = u0 + (s - s0) * (u1 - u0) / (s1 - s0);
            u = Math.Clamp(u, u0, u1);

            // Refine in the local segment using Newton + fallback to bisection
            double lo = u0;
            double hi = u1;
            for (int iter = 0; iter < maxIterations; iter++)
            {
                double segLen = IntegrateArcLength(u0, u);
                double f = (s0 + segLen) - s;

                if (Math.Abs(f) <= tolerance)
                    return u;

                // Maintain bracket
                if (f > 0)
                    hi = u;
                else
                    lo = u;

                Vector3Double d1 = CurveEvaluator.EvaluateFirstDerivative(_curve, u);
                double speed = d1.magnitude;

                double uNew;
                if (speed > 1e-12)
                {
                    // Newton step
                    uNew = u - f / speed;
                }
                else
                {
                    // If speed is near-zero, Newton is unreliable.
                    uNew = 0.5 * (lo + hi);
                }

                // Clamp to bracket
                if (uNew <= lo || uNew >= hi || double.IsNaN(uNew) || double.IsInfinity(uNew))
                {
                    uNew = 0.5 * (lo + hi);
                }

                if (Math.Abs(uNew - u) <= tolerance * Math.Max(1.0, (u1 - u0)))
                    return uNew;

                u = uNew;
            }

            return u;
        }

        private int FindSegmentByU(double u)
        {
            int idx = Array.BinarySearch(_us, u);
            if (idx >= 0)
                return idx;

            idx = ~idx - 1;
            if (idx < 0)
                return 0;
            if (idx >= _us.Length - 1)
                return _us.Length - 2;
            return idx;
        }

        private int FindSegmentByS(double s)
        {
            int idx = Array.BinarySearch(_ss, s);
            if (idx >= 0)
                return idx;

            idx = ~idx - 1;
            if (idx < 0)
                return 0;
            if (idx >= _ss.Length - 1)
                return _ss.Length - 2;
            return idx;
        }

        private double IntegrateArcLength(double startU, double endU)
        {
            if (endU <= startU)
                return 0.0;

            // 5-point Gauss-Legendre on [startU, endU]
            double half = 0.5 * (endU - startU);
            double center = 0.5 * (startU + endU);

            double sum = 0.0;
            for (int k = 0; k < GaussNode5.Length; k++)
            {
                double xi = GaussNode5[k];
                double wi = GaussWeight5[k];
                double u = center + half * xi;

                Vector3Double d = CurveEvaluator.EvaluateFirstDerivative(_curve, u);
                sum += wi * d.magnitude;
            }

            return half * sum;
        }

        internal static CurveArcLengthParameterization Build(NurbsCurve curve, int subdivisionsPerSpan)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            if (subdivisionsPerSpan < 1)
                throw new ArgumentOutOfRangeException(nameof(subdivisionsPerSpan), "subdivisionsPerSpan must be >= 1.");

            int degree = curve.Degree;
            double[] knots = curve.KnotVector.Knots;
            if (knots == null || knots.Length < 2)
                return new CurveArcLengthParameterization(curve, 0.0, 0.0, Array.Empty<double>(), Array.Empty<double>());

            double uMin = knots[degree];
            double uMax = knots[knots.Length - degree - 1];

            if (uMax <= uMin)
                return new CurveArcLengthParameterization(curve, uMin, uMax, new[] { uMin }, ZeroArcLengthTable);

            var us = new List<double>();
            var ss = new List<double>();

            us.Add(uMin);
            ss.Add(0.0);

            double acc = 0.0;

            for (int i = 0; i < knots.Length - 1; i++)
            {
                double a = knots[i];
                double b = knots[i + 1];
                if (b <= a)
                    continue;

                // Clip to evaluation domain
                a = Math.Max(a, uMin);
                b = Math.Min(b, uMax);
                if (b <= a)
                    continue;

                for (int div = 0; div < subdivisionsPerSpan; div++)
                {
                    double u0 = a + (b - a) * div / subdivisionsPerSpan;
                    double u1 = a + (b - a) * (div + 1) / subdivisionsPerSpan;

                    // Ensure increasing and within [uMin, uMax]
                    u0 = Math.Clamp(u0, uMin, uMax);
                    u1 = Math.Clamp(u1, uMin, uMax);
                    if (u1 <= u0)
                        continue;

                    // Append u1; u0 is already last
                    double segLen = IntegrateArcLengthStatic(curve, u0, u1);
                    acc += segLen;

                    us.Add(u1);
                    ss.Add(acc);
                }
            }

            // Ensure last is uMax
            if (us.Count == 0 || us[^1] < uMax)
            {
                double lastU = us.Count == 0 ? uMin : us[^1];
                double lastS = ss.Count == 0 ? 0.0 : ss[^1];
                double tail = IntegrateArcLengthStatic(curve, lastU, uMax);
                us.Add(uMax);
                ss.Add(lastS + tail);
            }
            else
            {
                us[^1] = uMax;
            }

            return new CurveArcLengthParameterization(curve, uMin, uMax, us.ToArray(), ss.ToArray());
        }

        private static double IntegrateArcLengthStatic(NurbsCurve curve, double startU, double endU)
        {
            if (endU <= startU)
                return 0.0;

            double half = 0.5 * (endU - startU);
            double center = 0.5 * (startU + endU);

            double sum = 0.0;
            for (int k = 0; k < GaussNode5.Length; k++)
            {
                double xi = GaussNode5[k];
                double wi = GaussWeight5[k];
                double u = center + half * xi;

                Vector3Double d = CurveEvaluator.EvaluateFirstDerivative(curve, u);
                sum += wi * d.magnitude;
            }

            return half * sum;
        }
    }
}
