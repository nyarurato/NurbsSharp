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
    }
}
