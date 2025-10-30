using System.ComponentModel;
using System.Linq;
using System.Numerics;
using System;

namespace NurbsSharp.Core
{
    public class KnotVector
    {
        public double[] Knots { get; set; }
        public int Length => Knots.Length;
        public KnotVector()
        {
            Knots = new double[] { };
        }

        public KnotVector(double[] knots)
        {
            Knots = knots;
        }

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

        public override string ToString()
        {
            return $"KnotVector[{Length}] {{ {string.Join(", ", Knots)} }}";
        }

        /// <summary>
        /// (en) Generates a uniform knot vector.
        /// (ja) 均等分布のノットベクトルを生成します。
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
