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
            // IGES 126 (Rational B-spline curve) format:
            // K,M,PROP1,PROP2,PROP3,PROP4,N+1 knots,N+1 weights,N+1 control points (X,Y,Z),V0,V1
            // K = Upper index of sum (number of control points - 1)
            // M = Degree of basis functions
            // PROP1 = 0: non-planar, 1: planar
            // PROP2 = 0: not closed, 1: closed
            // PROP3 = 0: not periodic, 1: periodic
            // PROP4 = 0: rational, 1: polynomial (non-rational)
            int D_pointer = 1; // Directory Entry pointer
            string D_pointer_str = D_pointer.ToString().PadLeft(8, ' ');

            int K = _curve.ControlPoints.Length - 1;
            int M = _curve.Degree;
            int PROP1 = 0; // non-planar
            int PROP2 = 0; // not closed
            int PROP3 = 0; // not periodic
            int PROP4 = 0; // rational

            StringBuilder sb = new StringBuilder();
            sb.Append($"{EntityType},{K},{M},{PROP1},{PROP2},{PROP3},{PROP4},");

            const int paramater_max_per_line = 63;
            int line_counter = 40; // approximate starting position

            // Knot vector (K+M+2 values)
            foreach (var knot in _curve.KnotVector.Knots)
            {
                var knotStr = $"{knot:F10}".TrimEnd('0').TrimEnd('.');
                if (line_counter + knotStr.Length + 1 > paramater_max_per_line)
                {
                    sb.AppendLine();
                    line_counter = 0;
                }
                sb.Append($"{knotStr},");
                line_counter += knotStr.Length + 1;
            }

            // Weights (K+1 values)
            foreach (var cp in _curve.ControlPoints)
            {
                var weightStr = $"{cp.Weight:F10}".TrimEnd('0').TrimEnd('.');
                if (line_counter + weightStr.Length + 1 > paramater_max_per_line)
                {
                    sb.AppendLine();
                    line_counter = 0;
                }
                sb.Append($"{weightStr},");
                line_counter += weightStr.Length + 1;
            }

            // Control points (K+1 triplets of X,Y,Z)
            foreach (var cp in _curve.ControlPoints)
            {
                foreach (var coord in new[] { cp.Position.X, cp.Position.Y, cp.Position.Z })
                {
                    var coordStr = $"{coord:F10}".TrimEnd('0').TrimEnd('.');
                    if (line_counter + coordStr.Length + 1 > paramater_max_per_line)
                    {
                        sb.AppendLine();
                        line_counter = 0;
                    }
                    sb.Append($"{coordStr},");
                    line_counter += coordStr.Length + 1;
                }
            }

            // Parameter range V(0) and V(1)
            double V0 = _curve.KnotVector.Knots[0];
            double V1 = _curve.KnotVector.Knots[_curve.KnotVector.Length - 1];
            sb.Append($"{V0:F10},{V1:F10};");

            ParameterData = sb.ToString()
                              .Split(Environment.NewLine)
                              .Select(s => s.PadRight(64, ' ') + D_pointer_str)
                              .ToArray();
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

            string status = "00010000";//Visible:00,Physically Dependent:00,Geometry:00, GlobalTopDown:00
            string zerostr = "       0";
            string nonestr = "        ";
            
            string s1 = $"     {EntityType}{parameterPointer.ToString().PadLeft(8, ' ')}{zerostr}{zerostr}{zerostr}{nonestr}{nonestr}{nonestr}{status}";
            string s2 = $"     {EntityType}{zerostr}{zerostr}{parameterLineCount.ToString().PadLeft(8, ' ')}{zerostr}{nonestr}{nonestr}{nonestr}{zerostr}";
            return new[] { s1, s2 };
        }
    }
}
