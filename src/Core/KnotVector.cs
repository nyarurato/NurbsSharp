using System.ComponentModel;
using System.Linq;
using System.Numerics;
using System;

namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) knot vector used in NURBS
    /// (ja) NURBSで使用されるノットベクトル
    /// </summary>
    public class KnotVector
    {
        /// <summary>
        /// (en) Indicates whether the knot vector is clamped
        /// (ja) ノットベクトルがクランプされているかどうかを示す
        /// </summary>
        public bool IsClamped { get; private set; } = true;
        /// <summary>
        /// (en) Indicates whether the knot vector is closed
        /// (ja) ノットベクトルが閉じているかどうかを示す
        /// </summary>
        //TODO: 対応未実装
        public bool IsClosed { get; private set; } = false;
        /// <summary>
        /// Knot vector like {0,0,0,0.5,0.5,1,1,1}
        /// </summary>
        public double[] Knots { get; private set; }
        /// <summary>
        /// Knot vector length
        /// </summary>
        public int Length => Knots.Length;

        /// <summary>
        /// Constructor
        /// </summary>
        public KnotVector()
        {
            Knots = [];
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="knots"></param>
        /// <param name="degree"></param>
        public KnotVector(double[] knots, int degree)
        {
            Knots = knots;
            Validate();
            TypeValidate(degree);
        }


        /// <summary>
        /// (en) Validates the knot vector
        /// (ja) ノットベクトルを検証
        /// </summary>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        private bool Validate()
        {
            if (Knots == null)
                throw new ArgumentNullException(nameof(Knots),"Knot vector is null");
            if(Knots.Length == 0)
                throw new ArgumentNullException(nameof(Knots), "Knot vector is empty.");

            for (int i = 1; i < Knots.Length; i++)
            {
                if (Knots[i] < Knots[i - 1])
                    throw new ArgumentException("Knot vector is not non-decreasing.");
            }
            return true;
        }

        /// <summary>
        /// (en) Validates the knot vector type based on the degree
        /// (ja) ノットベクトルの種類を次数に基づいて検証
        /// </summary>
        /// <param name="degree"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        /// <exception cref="InvalidOperationException"></exception>
        private bool TypeValidate(int degree)
        {
            if (IsClamped)
            {
                int expectedMultiplicity = degree + 1;
                if (Knots.Length < 2 * expectedMultiplicity)
                    throw new ArgumentException("Knot vector length is too short for clamped type.");
                // Check start multiplicity
                double firstKnot = Knots[0];
                int startMultiplicity = Knots.TakeWhile(k => k == firstKnot).Count();
                if (startMultiplicity != expectedMultiplicity)
                    throw new InvalidOperationException("Knot vector START multiplicity does not match clamped type.");
                // Check end multiplicity
                double lastKnot = Knots[^1];
                int endMultiplicity = Knots.AsEnumerable().Reverse().TakeWhile(k => k == lastKnot).Count();
                if (endMultiplicity != expectedMultiplicity)
                    throw new InvalidOperationException("Knot vector END multiplicity does not match clamped type.");
            }
            
            return true;
        }

        /// <summary>
        /// (en) Normalizes the knot vector to the range [0, 1]
        /// (ja) ノットベクトルを [0, 1] の範囲に正規化
        /// </summary>
        /// <exception cref="InvalidOperationException"></exception>
        public void Normalize()
        {
            double min = Knots[0];
            double max = Knots[^1];
            double range = max - min;
            if (range == 0)
                throw new InvalidOperationException("Cannot normalize knot vector with zero range.");
            double[] normalizedKnots = Knots.Select(k => (k - min) / range).ToArray();
            Knots = normalizedKnots;
            Validate();
        }

        /// <summary>
        /// returns a string representation of the knot vector
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"KnotVector[{Length}] {{ {string.Join(", ", Knots)} }}";
        }

        /// <summary>
        /// (en) Generates a uniform knot vector
        /// (ja) 均等分布のノットベクトルを生成
        /// 6 -> {0, 0.2, 0.4, 0.6, 0.8, 1.0}
        /// </summary>
        /// <param name="length"></param>
        /// <returns></returns>
        public static KnotVector GetUniformKnot(int length)
        {
            return new KnotVector
            {
                Knots = Enumerable.Range(0, length).Select(i => (double)i / (length - 1)).ToArray()
            };
        }

        /// <summary>
        /// (en) Generates a clamped knot vector based on degree and number of control points
        /// (ja) 次数と制御点の数に基づいてクランプされたノットベクトルを生成 
        /// (example: degree=2, numControlPoints=5 -> {0,0,0,0.333,0.666,1,1,1})
        /// max multiplicity = degree + 1, min multiplicity = 1
        /// </summary>
        /// <param name="degree"></param>
        /// <param name="numControlPoints"></param>
        /// <param name="multiplicity"></param>
        /// <returns></returns>
        public static KnotVector GetClampedKnot(int degree, int numControlPoints, int? multiplicity=null)
        {
            int mult = multiplicity ?? (degree + 1);
            if (mult < 0)
                throw new ArgumentOutOfRangeException(nameof(multiplicity), "Multiplicity cannot be negative.");

            if (mult > degree + 1)
                throw new ArgumentOutOfRangeException(nameof(multiplicity), "Multiplicity cannot be greater than degree + 1 for clamped knot vector.");

            //TODO: Support for multiplicity less than degree + 1
            if (mult != degree + 1)
                throw new NotSupportedException("Currently, only multiplicity equal to degree + 1 is supported for clamped knot vector.");

            int n = numControlPoints;
            int m = n + degree + 1;
            
            if (m < 2 * mult)
                throw new ArgumentOutOfRangeException(nameof(multiplicity),"Multiplicity is too big for knot vector size");

            double[] knots = new double[m];
            // Start clamping
            for (int i = 0; i < mult; i++)
            {
                knots[i] = 0.0;
            }
            // Internal knots
            int internalKnotsCount = m - 2 * mult;
            for (int i = 1; i <= internalKnotsCount; i++)
            {
                knots[degree + i] = (double)i / (internalKnotsCount + 1);
            }
            // End clamping
            for (int i = m - mult; i < m; i++)
            {
                knots[i] = 1.0;
            }
            return new KnotVector(knots, degree);
        }
    }
}
