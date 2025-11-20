using System;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Generation.Interpolation
{
    /// <summary>
    /// (en) Helper methods for curve interpolation
    /// (ja) 曲線補間のヘルパーメソッド
    /// </summary>
    internal static class InterpolationHelper
    {
        /// <summary>
        /// (en) Computes parameter values for interpolation points
        /// (ja) 補間点のパラメータ値を計算
        /// </summary>
        /// <param name="points">Data points</param>
        /// <param name="paramType">Parameterization type</param>
        /// <returns>Parameter values in [0, 1]</returns>
        public static double[] ComputeParameters(Vector3Double[] points, ParameterizationType paramType)
        {
            int n = points.Length - 1;
            double[] u = new double[n + 1];

            u[0] = 0.0;
            u[n] = 1.0;

            if (n == 1) return u;

            // Compute chord lengths or other metrics
            double[] distances = new double[n];
            double totalDistance = 0.0;

            for (int i = 0; i < n; i++)
            {
                double dist = points[i + 1].DistanceTo(points[i]);
                
                switch (paramType)
                {
                    case ParameterizationType.Uniform:
                        distances[i] = 1.0;
                        break;
                    case ParameterizationType.Chord:
                        distances[i] = dist;
                        break;
                    case ParameterizationType.Centripetal:
                        distances[i] = Math.Sqrt(dist);
                        break;
                    default:
                        distances[i] = dist;
                        break;
                }
                totalDistance += distances[i];
            }

            // Normalize to [0, 1]
            double cumulative = 0.0;
            for (int i = 1; i < n; i++)
            {
                cumulative += distances[i - 1];
                u[i] = cumulative / totalDistance;
            }

            return u;
        }

        /// <summary>
        /// (en) Generates a clamped knot vector using averaging method
        /// (ja) 平均化法によるクランプノットベクトルを生成
        /// </summary>
        ///  <param name="n">Number of data points - 1</param>
        /// <param name="p">Degree</param>
        /// <param name="parameters">Parameter values</param>
        /// <returns>Knot vector</returns>
        public static double[] AveragingKnotVector(int n, int p, double[] parameters)
        {
            int m = n + p + 1;
            double[] knots = new double[m + 1];

            // Clamp the ends
            for (int i = 0; i <= p; i++)
            {
                knots[i] = 0.0;
                knots[m - i] = 1.0;
            }

            // Internal knots using averaging
            for (int j = 1; j <= n - p; j++)
            {
                double sum = 0.0;
                for (int i = j; i <= j + p - 1; i++)
                {
                    sum += parameters[i];
                }
                knots[p + j] = sum / p;
            }

            return knots;
        }

        /// <summary>
        /// (en) Builds the basis function matrix for interpolation
        /// (ja) 補間用の基底関数行列を構築
        /// </summary>
        /// <param name="n">Number of data points - 1</param>
        /// <param name="p">Degree</param>
        /// <param name="knots">Knot vector</param>
        /// <param name="parameters">Parameter values</param>
        /// <returns>Basis matrix N[i][j] = N_{j,p}(u[i])</returns>
        public static double[][] BuildBasisMatrix(int n, int p, double[] knots, double[] parameters)
        {
            double[][] N = new double[n + 1][];
            for (int i = 0; i <= n; i++)
            {
                N[i] = new double[n + 1];
                for (int j = 0; j <= n; j++)
                {
                    N[i][j] = BasicEvaluator.BSplineBasisFunction(j, p, parameters[i], knots);
                }
            }
            return N;
        }
    }
}
