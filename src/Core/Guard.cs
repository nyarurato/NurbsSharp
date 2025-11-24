using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NurbsSharp.Core
{
    internal static class Guard
    {
        /// <summary>
        /// (en) Throws an ArgumentNullException if the provided value is null.
        /// (ja) 指定された値がnullの場合、ArgumentNullExceptionをスローします。
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="value"></param>
        /// <param name="paramName"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public static void ThrowIfNull<T>(T? value, string paramName) where T : class
        {
#if NET8_OR_GREATER
            ArgumentNullException.ThrowIfNull(value, paramName);
#else
            if (value == null)
                throw new ArgumentNullException(paramName, $"{paramName} cannot be null.");
#endif
        }

        /// <summary>
        /// (en) Throws an ArgumentOutOfRangeException if the provided integer value is negative or zero.
        /// (ja) 指定された整数値が負またはゼロの場合、ArgumentOutOfRangeExceptionをスローします。
        /// </summary>
        /// <param name="value"></param>
        /// <param name="paramName"></param>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static void ThrowIfNegativeOrZero(int value, string paramName)
        {
#if NET8_OR_GREATER
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, paramName);

#else
            if (value <= 0)
                throw new ArgumentOutOfRangeException(paramName, $"{paramName} must be greater than zero.");
#endif
        }

        /// <summary>
        /// (en) Throws an ArgumentOutOfRangeException if the provided double value is negative.
        /// (ja) 指定された倍精度浮動小数点値が負の場合、ArgumentOutOfRangeExceptionをスローします。
        /// </summary>
        /// <param name="value"></param>
        /// <param name="paramName"></param>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static void ThrowIfNegativeOrZero(double value, string paramName)
        {
#if NET8_OR_GREATER
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, paramName);

#else
            if (value < 0)
                throw new ArgumentOutOfRangeException(paramName, $"{paramName} cannot be negative.");
#endif
        }

        /// <summary>
        /// (en) Throws an ArgumentOutOfRangeException if the provided integer value is negative.
        /// (ja) 指定された整数値が負の場合、ArgumentOutOfRangeExceptionをスローします。
        /// </summary>
        /// <param name="value"></param>
        /// <param name="paramName"></param>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static void ThrowIfNegative(int value, string paramName)
        {
#if NET8_OR_GREATER
            ArgumentOutOfRangeException.ThrowIfNegative(value, paramName);

#else
            if (value < 0)
                throw new ArgumentOutOfRangeException(paramName, $"{paramName} must be greater than zero.");
#endif
        }
    }
}
