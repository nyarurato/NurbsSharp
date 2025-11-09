using System.ComponentModel;
using System.Linq;
using System.Numerics;
using System;

namespace NurbsSharp.Core
{
    //TODO: Check KnotVector types
    public enum KnotVectorType
    {
        UNIFORM,
        CLAMPED,
        CLOSED
    }

    /// <summary>
    /// (en) knot vector used in NURBS
    /// (ja) NURBSで使用されるノットベクトル
    /// </summary>
    public class KnotVector
    {
        public KnotVectorType Type { get; set; } = KnotVectorType.UNIFORM;//TODO: 対応未実装
        public double[] Knots { get; set; }
        public int Length => Knots.Length;

        /// <summary>
        /// Constructor
        /// </summary>
        public KnotVector()
        {
            Knots = new double[] { };
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="knots"></param>
        public KnotVector(double[] knots)
        {
            Knots = knots;
            Validate();
        }

        /// <summary>
        /// (en) Validates the knot vector
        /// (ja) ノットベクトルを検証
        /// </summary>
        /// <returns></returns>
        /// <exception cref="Exception"></exception>
        public bool Validate()
        {
            if (Knots == null || Knots.Length == 0)
                throw new Exception("Knot vector is null or empty.");

            for (int i = 1; i < Knots.Length; i++)
            {
                if (Knots[i] < Knots[i - 1])
                    throw new Exception("Knot vector is not non-decreasing.");
            }
            return true;
        }

        public void Normalize()
        {
            double min = Knots[0];
            double max = Knots[Knots.Length - 1];
            double range = max - min;
            if (range == 0)
                throw new Exception("Cannot normalize knot vector with zero range.");
            double[] normalizedKnots = Knots.Select(k => (k - min) / range).ToArray();
        }


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
        public static KnotVector GetUniformKnot(int length) {
            return new KnotVector {
                Knots = Enumerable.Range(0, length).Select(i => (double)i/(length-1)).ToArray()
            };
        }
    }
}
