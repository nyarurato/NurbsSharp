using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Analysis
{
    /// <summary>
    /// (en) Basic analysis utilities for NURBS
    /// (ja) NURBSの基本的な解析ユーティリティクラス
    /// </summary>
    public class BasicAnalyzer
    {
                /// <summary>
        /// (en) 5-point Gauss-Legendre quadrature nodes
        /// (ja) 5点ガウス・ルジャンドル求積法のノード
        /// </summary>
        protected static readonly double[] GaussNode5 = [
            -0.906179845938664,
            -0.538469310105683,
            0,
            0.538469310105683,
            0.906179845938664
        ];
        /// <summary>
        /// (en) 5-point Gauss-Legendre quadrature weights
        /// (ja) 5点ガウス・ルジャンドル求積法の重み
        /// </summary>
        protected static readonly double[] GaussWeight5 = [
            0.236926885056189,
            0.478628670499366,
            0.568888888888889,
            0.478628670499366,
            0.236926885056189
        ];
    }
}