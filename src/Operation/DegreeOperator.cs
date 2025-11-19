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
    /// (en) Operator for degree elevation and reduction of NURBS
    /// (ja) NURBSの次数昇降のためのオペレーター
    /// </summary>
    public class DegreeOperator
    {
        //TODO: implement piegl's algorithm for degree elevation/reduction

        /// <summary>
        /// (en) Elevates the degree of the given NURBS curve by t while preserving its shape
        /// (ja) 形状を保つように与えられたNURBS曲線の次数をtだけ昇降します
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        /// <exception cref="NotImplementedException"></exception>
        public static NurbsCurve ElevateDegree(NurbsCurve curve, int t)
        {
            /*
            *  least square fitting method
            *  NOT piegl's algorithm
            */
            Guard.ThrowIfNull(curve, nameof(curve));
            if (t <= 0)
                return curve;

            int pOld = curve.Degree;
            int pNew = pOld + t;
            var oldCP = curve.ControlPoints;
            int nOld = oldCP.Length;
            int nNew = nOld + t;

            // create new clamped knot vector
            var newKnotVec = KnotVector.GetClampedKnot(pNew, nNew);

            // sampling for least-squares
            int sampleCount = Math.Max(nNew * 10, 200);
            double uMin = curve.KnotVector.Knots[pOld];
            double uMax = curve.KnotVector.Knots[curve.KnotVector.Length - pOld - 1];

            var samples = new List<Vector4Double>(sampleCount);
            var us = new double[sampleCount];
            for (int i = 0; i < sampleCount; i++)
            {
                double u = uMin + (uMax - uMin) * i / (sampleCount - 1);
                us[i] = u;
                samples.Add(EvaluateHomogeneous(curve, u));
            }

            // Build A matrix (sampleCount x nNew) of basis functions for new degree/knot
            double[][] A = new double[sampleCount][];
            for (int i = 0; i < sampleCount; i++)
            {
                double u = us[i];
                A[i] = new double[nNew];
                for (int j = 0; j < nNew; j++)
                {
                    A[i][j] = BSplineBasisFunction(j, pNew, u, newKnotVec.Knots);
                }
            }

            // Build normal equations ATA and ATb for x,y,z
            // Build normal equations ATA and ATb for homogeneous components (Xw, Yw, Zw, W)
            double[][] ATA = new double[nNew][];
            double[] ATbXw = new double[nNew];
            double[] ATbYw = new double[nNew];
            double[] ATbZw = new double[nNew];
            double[] ATbW = new double[nNew];

            for (int i = 0; i < nNew; i++)
            {
                ATA[i] = new double[nNew];
                for (int j = 0; j < nNew; j++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < sampleCount; k++)
                        sum += A[k][i] * A[k][j];
                    ATA[i][j] = sum;
                }
                double sumXw = 0.0, sumYw = 0.0, sumZw = 0.0, sumW = 0.0;
                for (int k = 0; k < sampleCount; k++)
                {
                    var h = samples[k];
                    sumXw += A[k][i] * h.X;
                    sumYw += A[k][i] * h.Y;
                    sumZw += A[k][i] * h.Z;
                    sumW += A[k][i] * h.W;
                }
                ATbXw[i] = sumXw;
                ATbYw[i] = sumYw;
                ATbZw[i] = sumZw;
                ATbW[i] = sumW;
            }

            // Solve ATA * X = ATb for each homogeneous component
            double[] solXw = SolveLinearSystem(ATA, ATbXw);
            double[] solYw = SolveLinearSystem(ATA, ATbYw);
            double[] solZw = SolveLinearSystem(ATA, ATbZw);
            double[] solW = SolveLinearSystem(ATA, ATbW);

            // Build control points from homogeneous solution
            ControlPoint[] newCP = new ControlPoint[nNew];
            for (int i = 0; i < nNew; i++)
            {
                double w = solW[i];
                if (LinAlg.ApproxEqual(w, 0.0))
                    throw new InvalidOperationException("ElevateDegree: solved weight is zero for a control point.");
                newCP[i] = new ControlPoint(solXw[i] / w, solYw[i] / w, solZw[i] / w, w);
            }

            return new NurbsCurve(pNew, newKnotVec, newCP);
        }
        /// <summary>
        /// (en) Reduces the degree of the given NURBS curve by t while approximating within the specified tolerance
        /// (ja) 指定された許容誤差内で近似しながら、与えられたNURBS曲線の次数をtだけ降下させます
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="t"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        /// <exception cref="NotImplementedException"></exception>
        public static NurbsCurve ReduceDegree(NurbsCurve curve, int t, double tolerance)
        {
            /*
            *  least square fitting method
            *  NOT piegl's algorithm
            */

            Guard.ThrowIfNull(curve, nameof(curve));
            if (t <= 0)
                return curve;

            int pOld = curve.Degree;
            int pNew = pOld - t;
            if (pNew < 1)
                throw new ArgumentOutOfRangeException(nameof(t), "Resulting degree must be at least 1.");

            var oldCP = curve.ControlPoints;
            int nOld = oldCP.Length;
            int nNew = nOld - t;
            if (nNew < pNew + 1)
                throw new ArgumentOutOfRangeException(nameof(t), "Not enough control points to reduce degree by given t.");

            var newKnotVec = KnotVector.GetClampedKnot(pNew, nNew);

            int sampleCount = Math.Max(nNew * 10, 200);
            double uMin = curve.KnotVector.Knots[pOld];
            double uMax = curve.KnotVector.Knots[curve.KnotVector.Length - pOld - 1];

            var samples = new List<Vector4Double>(sampleCount);
            var us = new double[sampleCount];
            for (int i = 0; i < sampleCount; i++)
            {
                double u = uMin + (uMax - uMin) * i / (sampleCount - 1);
                us[i] = u;
                samples.Add(EvaluateHomogeneous(curve, u));
            }

            double[][] A = new double[sampleCount][];
            for (int i = 0; i < sampleCount; i++)
            {
                double u = us[i];
                A[i] = new double[nNew];
                for (int j = 0; j < nNew; j++)
                {
                    A[i][j] = BSplineBasisFunction(j, pNew, u, newKnotVec.Knots);
                }
            }

            double[][] ATA = new double[nNew][];
            double[] ATbXw = new double[nNew];
            double[] ATbYw = new double[nNew];
            double[] ATbZw = new double[nNew];
            double[] ATbW = new double[nNew];

            for (int i = 0; i < nNew; i++)
            {
                ATA[i] = new double[nNew];
                for (int j = 0; j < nNew; j++)
                {
                    double sum = 0.0;
                    for (int k = 0; k < sampleCount; k++)
                        sum += A[k][i] * A[k][j];
                    ATA[i][j] = sum;
                }
                double sumXw = 0.0, sumYw = 0.0, sumZw = 0.0, sumW = 0.0;
                for (int k = 0; k < sampleCount; k++)
                {
                    var h = samples[k];
                    sumXw += A[k][i] * h.X;
                    sumYw += A[k][i] * h.Y;
                    sumZw += A[k][i] * h.Z;
                    sumW += A[k][i] * h.W;
                }
                ATbXw[i] = sumXw;
                ATbYw[i] = sumYw;
                ATbZw[i] = sumZw;
                ATbW[i] = sumW;
            }

            double[] solXw = SolveLinearSystem(ATA, ATbXw);
            double[] solYw = SolveLinearSystem(ATA, ATbYw);
            double[] solZw = SolveLinearSystem(ATA, ATbZw);
            double[] solW = SolveLinearSystem(ATA, ATbW);

            ControlPoint[] newCP = new ControlPoint[nNew];
            for (int i = 0; i < nNew; i++)
            {
                double w = solW[i];
                if (LinAlg.ApproxEqual(w, 0.0))
                    throw new InvalidOperationException("ReduceDegree: solved weight is zero for a control point.");
                newCP[i] = new ControlPoint(solXw[i] / w, solYw[i] / w, solZw[i] / w, w);
            }

            var newCurve = new NurbsCurve(pNew, newKnotVec, newCP);

            // Verify approximation error
            double maxErr = 0.0;
            int verifySamples = Math.Max(100, sampleCount);
            for (int i = 0; i < verifySamples; i++)
            {
                double u = uMin + (uMax - uMin) * i / (verifySamples - 1);
                var orig = CurveEvaluator.Evaluate(curve, u);
                var approx = CurveEvaluator.Evaluate(newCurve, u);
                double d = orig.DistanceTo(approx);
                if (d > maxErr) maxErr = d;
            }

            if (maxErr > tolerance)
                throw new InvalidOperationException($"Degree reduction exceeded tolerance. Max error={maxErr}");

            return newCurve;
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

        // Solve linear system Ax = b using Gaussian elimination with partial pivoting
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
                // find pivot
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

                // swap
                if (maxRow != k)
                {
                    for (int j = k; j < n + 1; j++)
                    {
                        double tmp = M[k][j];
                        M[k][j] = M[maxRow][j];
                        M[maxRow][j] = tmp;
                    }
                }

                // eliminate
                for (int i = k + 1; i < n; i++)
                {
                    double f = M[i][k] / M[k][k];
                    for (int j = k; j < n + 1; j++)
                        M[i][j] -= f * M[k][j];
                }
            }

            // back substitution
            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double s = M[i][n];
                for (int j = i + 1; j < n; j++) s -= M[i][j] * x[j];
                x[i] = s / M[i][i];
            }
            return x;
        }

        // Evaluate homogeneous point H(u) = sum_i N_{i,p}(u) * P_i^w  (returns Vector4Double (Xw, Yw, Zw, W))
        private static Vector4Double EvaluateHomogeneous(NurbsSharp.Geometry.NurbsCurve curve, double u)
        {
            Guard.ThrowIfNull(curve, nameof(curve));
            int p = curve.Degree;
            var knots = curve.KnotVector.Knots;
            var cps = curve.ControlPoints;

            // clamp u to valid evaluation domain
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
