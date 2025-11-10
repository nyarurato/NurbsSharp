using NurbsSharp.IO.IGES;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;


namespace NurbsSharp.IO.IGES
{
    /// <summary>
    /// (en)IGES exporter for Rational B-spline curve (Entity Type 126)
    /// (ja)Rational B-spline 曲線（エンティティタイプ 126）のIGESエクスポーター
    /// </summary>
    public class IgesRationalBSplineCurve : IIgesExportEntity
    {
        private readonly NurbsCurve _curve;
        /// <summary>
        /// Entity Type Identifier
        /// </summary>
        public int EntityType => 126;
        /// <summary>
        /// Parameter Entity Data
        /// </summary>
        public string[] ParameterData { get; private set; }
        /// <summary>
        /// Parameter Line Count
        /// </summary>
        public int ParameterLineCount => ParameterData.Length;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="curve"></param>
        public IgesRationalBSplineCurve(NurbsCurve curve) { 
            _curve = curve; 
            ParameterData = Array.Empty<string>();
        }

        /// <summary>
        /// (en) Generates the parameter data for the IGES.
        /// (ja) IGESのパラメータデータを生成します。
        /// </summary>
        /// <returns></returns>
        public string[] GenerateParameterData()
        {
            // IGES 126 (Rational B-spline curve) の主要フィールド（簡易）
            // 実装方針: JAMA-IS で必要なパラメータのみ列挙
            // フォーマット（簡易）: degree, m, n, knotList..., weights..., controlPoints...
            int p = _curve.Degree;
            int n = _curve.ControlPoints.Length - 1; // n = controlPointCount -1
            var knots = _curve.KnotVector.Knots;
            string knotsStr = string.Join(",", knots.Select(k => k.ToString("G17")));
            string weights = string.Join(",", _curve.ControlPoints.Select(cp => cp.Weight.ToString("G17")));
            string cps = string.Join(",", _curve.ControlPoints.Select(cp => $"{cp.Position.X:G17},{cp.Position.Y:G17},{cp.Position.Z:G17}"));
            // 最低限の情報を返す
            ParameterData = new[] { $"{p},{n},{knotsStr},{weights},{cps}" };
            return ParameterData;

        }

        /// <summary>
        /// (en) Generates the directory entry for the IGES.
        /// (ja) IGESのディレクトリエントリを生成します。
        /// </summary>
        /// <param name="parameterPointer"></param>
        /// <param name="parameterLineCount"></param>
        /// <returns></returns>
        /// <exception cref="InvalidOperationException"></exception>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public string[] GenerateDirectoryEntry(int parameterPointer, int parameterLineCount)
        {
            if (ParameterData.Length == 0)
                throw new InvalidOperationException("ParameterData is not generated yet.");

            if (parameterPointer <= 0)
                throw new ArgumentOutOfRangeException(nameof(parameterPointer));
            if (parameterLineCount <= 0)
                throw new ArgumentOutOfRangeException(nameof(parameterLineCount));

            string status = "01010000"; // placeholder
            string zerostr = "       0";
            string s1 = $"     {EntityType}{parameterPointer.ToString().PadLeft(7, ' ')}{zerostr}{zerostr}{zerostr}                        {status}D";
            string s2 = $"     {EntityType}{zerostr}{zerostr}{parameterLineCount.ToString().PadLeft(7, ' ')}{zerostr}                               0D";
            return new[] { s1, s2 };
        }
    }
}
