using NurbsSharp.IO.IGES;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Geometry;

namespace NurbsSharp.IO.IGES
{
    /// <summary>
    /// (en)IGES exporter for Rational B-spline surface (Entity Type 128)
    /// (ja)Rational B-spline サーフェス（エンティティタイプ 128）のIGESエクスポーター
    /// </summary>
    public class IgesRationalBSplineSurface : IIgesExportEntity
    {
        private readonly NurbsSurface _surf;
        /// <summary>
        /// Gets the entity type identifier.
        /// </summary>
        public int EntityType => 128;
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
        /// <param name="surf"></param>
        public IgesRationalBSplineSurface(NurbsSurface surf)
        {
            _surf = surf;
            ParameterData = Array.Empty<string>();
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

            string status = "01010000";
            string zerostr = "       0";
            string nonestr = "        ";

            string s1 = $"     {EntityType}{parameterPointer.ToString().PadLeft(8, ' ')}{zerostr}{zerostr}{zerostr}{nonestr}{nonestr}{nonestr}{status}";
            string s2 = $"     {EntityType}{zerostr}{zerostr}{parameterLineCount.ToString().PadLeft(8, ' ')}{zerostr}{nonestr}{nonestr}{nonestr}{zerostr}";
            return new[] { s1, s2 };
        }

        /// <summary>
        /// (en) Generates the parameter data for the IGES Rational B-spline surface.
        /// (ja) IGES Rational B-spline サーフェスのパラメータデータを生成します。
        /// </summary>
        /// <returns></returns>
        public string[] GenerateParameterData()
        {
            int D_pointer = 1; // placeholder
            string D_pointer_str = D_pointer.ToString().PadLeft(8, ' ');

            StringBuilder stringBuilder = new StringBuilder();
            // IGES 128 (Rational B-spline surface) の主要フィールド（簡易）
            //K1,K2,M1, M2, P1, P2, P3, P4, P5,
            int k1 = _surf.ControlPoints.Length - 1;//basis func sigma 0 to k1 (u)
            int k2 = _surf.ControlPoints[0].Length - 1;
            int m1 = _surf.DegreeU;
            int m2 = _surf.DegreeV;
            int p1 = 0;//1 = Closed in first parametric variable direction
            int p2 = 0;//1 = Closed in second parametric variable direction
            int p3 = 1;//1 = Polynomial surface (non-rational), 0 = Rational surface
            int p4 = 0;//1 = Periodic in first parametric variable direction
            int p5 = 0;//1 = Periodic in second parametric variable direction
            stringBuilder.AppendLine($"{EntityType},{k1},{k2},{m1},{m2},{p1},{p2},{p3},{p4},{p5},");

            int ustr_counter = 0;
            const int paramater_max_per_line = 63;

            //parameter 10 - 10+A : knot u vector
            foreach (var kv in _surf.KnotVectorU.Knots)
            {
                var ustr = $"{kv:F10}".TrimEnd('0');
                if (ustr_counter + ustr.Length > paramater_max_per_line)
                {
                    stringBuilder.AppendLine();
                    ustr_counter = 0;
                }
                stringBuilder.Append($"{ustr},");
                ustr_counter += ustr.Length + 1;
            }
            stringBuilder.AppendLine();

            //parameter 11+A - 11+A+B : knot v vector
            ustr_counter = 0;
            foreach (var kv in _surf.KnotVectorV.Knots)
            {
                var ustr = $"{kv:F10}".TrimEnd('0');
                if (ustr_counter + ustr.Length > paramater_max_per_line)
                {
                    stringBuilder.AppendLine();
                    ustr_counter = 0;
                }
                stringBuilder.Append($"{ustr},");
                ustr_counter += ustr.Length + 1;
            }
            stringBuilder.AppendLine();

            //parameter 12+A+B - 11+A+B+C : weights W(0,0)->W(1,0)->W(2,0)->... flattened U major then V
            ustr_counter = 0;
            for (int v = 0; v <= k2; v++)
            {
                for (int u = 0; u <= k1; u++)
                {
                    var wstr = $"{_surf.ControlPoints[u][v].Weight:F10}".TrimEnd('0');
                    if (ustr_counter + wstr.Length > paramater_max_per_line)
                    {
                        stringBuilder.AppendLine();
                        ustr_counter = 0;
                    }
                    stringBuilder.Append($"{wstr},");
                    ustr_counter += wstr.Length + 1;
                }
            }
            stringBuilder.AppendLine();

            //parameter 13+A+B+C - 11+A+B+4*C : control points X(0,0) -> Y(0,0) -> Z(0,0) -> X(1,0) ->... flattened U major then V
            ustr_counter = 0;
            for (int v = 0; v <= k2; v++)
            {
                for (int u = 0; u <= k1; u++)
                {
                    var cp = _surf.ControlPoints[u][v];
                    var xstr = $"{cp.Position.X:F10}".TrimEnd('0');
                    var ystr = $"{cp.Position.Y:F10}".TrimEnd('0');
                    var zstr = $"{cp.Position.Z:F10}".TrimEnd('0');
                    foreach (var coordStr in new[] { xstr, ystr, zstr })
                    {
                        if (ustr_counter + coordStr.Length > paramater_max_per_line)
                        {
                            stringBuilder.AppendLine();
                            ustr_counter = 0;
                        }
                        stringBuilder.Append($"{coordStr},");
                        ustr_counter += coordStr.Length + 1;
                    }
                }
            }
            stringBuilder.AppendLine();

            //parameter 12+A+B+4*C :Starting value for first parametric direction
            //parameter 13+A+B+4*C :Ending value for first parametric directio
            //parameter 14+A+B+4*C :Starting value for second parametric direction
            //parameter 15+A+B+4*C :Ending value for second parametric direction
            stringBuilder.Append($"1,0,1,0;");

            ParameterData = stringBuilder.ToString()
                                .Split(Environment.NewLine)
                                .Select(s => s.PadRight(64, ' ') + D_pointer_str)
                                .ToArray();
            return ParameterData;
        }
    }
}
