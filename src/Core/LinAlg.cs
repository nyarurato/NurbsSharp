using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NurbsSharp.Core
{
    /// <summary>
    /// Linear Algebra utilities
    /// </summary>
    internal class LinAlg
    {
        public const double EPSILON = 1e-12;
        public static bool ApproxEqual(double a, double b)
        {
            return Math.Abs(a - b) < EPSILON;
        }

        public static bool ApproxNotEqual(double a, double b)
        {
            return Math.Abs(a - b) >= EPSILON;
        }

        public static bool IsZero(double a)
        {
            return Math.Abs(a) < EPSILON;
        }

        public static bool IsNotZero(double a)
        {
            return Math.Abs(a) >= EPSILON;
        }

        public static double BitDecrement(double value)
        {
#if NET8_0_OR_GREATER
            return Math.BitDecrement(value);
#else
            if (double.IsNaN(value) || double.IsInfinity(value))
                return value;

            long bits = BitConverter.DoubleToInt64Bits(value);
            if (value > 0)
                bits--;
            else if (value < 0)
                bits++;
            else
                bits = -1; // smallest negative subnormal number
            return BitConverter.Int64BitsToDouble(bits);
#endif
        }

        public static double BitIncrement(double value)
        {
#if NET8_0_OR_GREATER
            return Math.BitIncrement(value);
#else
            if (double.IsNaN(value) || double.IsInfinity(value))
                return value;
            long bits = BitConverter.DoubleToInt64Bits(value);
            if (value > 0)
                bits++;
            else if (value < 0)
                bits--;
            else
                bits = 1; // smallest positive subnormal number
            return BitConverter.Int64BitsToDouble(bits);
#endif
        }

        /// <summary>
        /// (en) Calculate the binomial coefficient "nCk"
        /// (ja) 二項係数 "nCk" を計算します
        /// </summary>
        /// <param name="n"></param>
        /// <param name="k"></param>
        /// <returns></returns>
        public static double BinomialCoefficient(int n, int k)
        {
            if (k < 0 || k > n)
                return 0;
            if (k == 0 || k == n)
                return 1;
            k = Math.Min(k, n - k); // Take advantage of symmetry
            double c = 1;
            for (int i = 0; i < k; i++)
            {
                c = c * (n - i) / (i + 1);
            }
            return c;
        }

        /// <summary>
        /// (en) Solves the linear system Ax = b using Gaussian elimination with partial pivoting
        /// (ja) 部分ピボット選択付きガウスの消去法により連立一次方程式 Ax = b を解く
        /// </summary>
        /// <param name="A">Coefficient matrix (n x n)</param>
        /// <param name="b">Right-hand side vector (n)</param>
        /// <returns>Solution vector x</returns>
        /// <exception cref="InvalidOperationException">Thrown when the system is singular or ill-conditioned</exception>
        public static double[] SolveLinearSystem(double[][] A, double[] b)
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

            // Forward elimination with partial pivoting
            for (int k = 0; k < n; k++)
            {
                // Find pivot
                int maxRow = k;
                double maxVal = Math.Abs(M[k][k]);
                for (int i = k + 1; i < n; i++)
                {
                    double val = Math.Abs(M[i][k]);
                    if (val > maxVal)
                    {
                        maxVal = val;
                        maxRow = i;
                    }
                }
                if (ApproxEqual(maxVal, 0.0))
                    throw new InvalidOperationException("Linear system is singular or ill-conditioned.");

                // Swap rows
                if (maxRow != k)
                {
                    for (int j = k; j < n + 1; j++)
                    {
                        (M[maxRow][j], M[k][j]) = (M[k][j], M[maxRow][j]);
                    }
                }

                // Eliminate column
                for (int i = k + 1; i < n; i++)
                {
                    double f = M[i][k] / M[k][k];
                    for (int j = k; j < n + 1; j++)
                        M[i][j] -= f * M[k][j];
                }
            }

            // Back substitution
            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double s = M[i][n];
                for (int j = i + 1; j < n; j++)
                    s -= M[i][j] * x[j];
                x[i] = s / M[i][i];
            }
            return x;
        }
    }
}
