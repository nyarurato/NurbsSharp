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
    /// (en)IGES exporter for Rational point (Entity Type 116)
    /// (ja)Rational 点（エンティティタイプ 116）のIGESエクスポーター
    /// </summary>
    internal class IgesPoint : IIgesExportEntity
    {
        private readonly Vector3Double _point;
        /// <summary>
        /// Entity Type Identifier
        /// </summary>
        public int EntityType => 116;
        /// <summary>
        /// Parameter Entity Data
        /// </summary>
        public string[] ParameterData { get; private set; }
        /// <summary>
        /// Parameter Line Count
        /// </summary>
        public int ParameterLineCount => ParameterData.Length;
        public int DirectoryPointer { get; set; } = 1;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="point"></param>
        public IgesPoint(Vector3Double point) { 
            _point = point; 
            ParameterData = [];
        }

        /// <summary>
        /// (en) Generates the parameter data for the IGES.
        /// (ja) IGESのパラメータデータを生成します。
        /// </summary>
        /// <returns></returns>
        public string[] GenerateParameterData()
        {
            // IGES 116 format:
            // X,Y,Z,PTR
            string D_pointer_str = "{XXXXXX}";//D_pointer.ToString().PadLeft(8, ' ');

            double X = _point.X;
            double Y = _point.Y;
            double Z = _point.Z;
            int PTR = 0; // Pointer to the DE of the Subfigure Definition Entity specifying the display symbol or zero.


            ParameterData = [($"{EntityType},{X},{Y},{Z},{PTR};").PadRight(64, ' ') + D_pointer_str];
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

            Guard.ThrowIfNegativeOrZero(parameterPointer, nameof(parameterPointer));
            Guard.ThrowIfNegativeOrZero(parameterLineCount, nameof(parameterLineCount));

            string status = "00010000";//Visible:00,Physically Dependent:00,Geometry:00, GlobalTopDown:00
            string zerostr = "       0";
            string nonestr = "        ";
            
            string s1 = $"     {EntityType}{parameterPointer,8}{zerostr}{zerostr}{zerostr}{nonestr}{nonestr}{nonestr}{status}";
            string s2 = $"     {EntityType}{zerostr}{zerostr}{parameterLineCount,8}{zerostr}{nonestr}{nonestr}{nonestr}{zerostr}";
            return [s1, s2];
        }

        /// <summary>
        /// (en) Sets the directory pointer in the parameter data.
        /// (ja) パラメータデータ内のディレクトリポインタを設定します。
        /// </summary>
        /// <param name="pointer"></param>
        /// <returns></returns>
        public bool SetDirectoryPointerToParameterString(int pointer)
        {
            if (ParameterData.Length == 0)
                return false;
            Guard.ThrowIfNegativeOrZero(pointer, nameof(pointer));
            string pointerStr = pointer.ToString().PadLeft(8, ' ');

            ParameterData = ParameterData
                .Select(line => line.Replace("{XXXXXX}", pointerStr))
                .ToArray();

            return true;
        }
    }
}
