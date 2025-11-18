using NurbsSharp.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NurbsSharp.IO.IGES
{
    /// <summary>
    /// Interface for IGES import entities
    /// </summary>
    internal interface IIgesImportEntity : IIgesEntity
    {
        /// <summary>
        /// (en) The geometry represented by this IGES entity
        /// (ja) このIGESエンティティが表すジオメトリ
        /// </summary>
        IGeometry Geometry { get; }
        /// <summary>
        /// (en) Parses the parameter entity data from the given string.
        /// (ja) 指定された文字列からパラメータエンティティデータを解析します。
        /// </summary>
        /// <param name="entity_str"></param>
        /// <returns></returns>
        bool ParseParameterEntity(string[] entity_str);
        /// <summary>
        /// (en) Parses the directory entry data from the given string.
        /// (ja) 指定された文字列からディレクトリエントリデータを解析します。
        /// </summary>
        /// <param name="directory_entry_str"></param>
        /// <returns></returns>
        (bool result, int parameter_start_pointer, int parameter_row_count) ParseDirectoryEntry(string[] directory_entry_str);
    }
}
