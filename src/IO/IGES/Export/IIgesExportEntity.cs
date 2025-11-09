using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NurbsSharp.IO.IGES
{

    interface IIgesExportEntity:IIgesEntity
    {
        /// <summary>
        /// IGES Parameter Data セクションに記載する文字列（カンマ区切り、実装側で適切にformat）
        /// 例: for curve 126 -> "p1,p2,p3,..."
        /// </summary>
        string[] GenerateParameterData();
        string[] GenerateDirectoryEntry(int paramPointerindex, int paramLineCount);

        string[] ParameterData { get; }
        int ParameterLineCount { get; }
    }
}
