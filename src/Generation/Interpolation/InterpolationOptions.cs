using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NurbsSharp.Generation.Interpolation
{
    /// <summary>
    /// (en) Parameterization type for curve interpolation
    /// (ja) 曲線補間のパラメータ化タイプ
    /// </summary>
    public enum ParameterizationType
    {
        /// <summary>
        /// (en) Uniform spacing: u[i] = i / (n-1)
        /// (ja) 等間隔: u[i] = i / (n-1)
        /// </summary>
        Uniform,

        /// <summary>
        /// (en) Chord length parameterization
        /// (ja) 弦長パラメータ化
        /// </summary>
        Chord,

        /// <summary>
        /// (en) Centripetal parameterization
        /// (ja) 求心パラメータ化
        /// </summary>
        Centripetal
    }

    /// <summary>
    /// (en) Options for curve interpolation
    /// (ja) 曲線補間のオプション
    /// </summary>
    public class InterpolationOptions
    {
        /// <summary>
        /// (en) Parameterization type (default: Chord)
        /// (ja) パラメータ化タイプ（デフォルト: 弦長）
        /// </summary>
        public ParameterizationType ParameterizationType { get; set; } = ParameterizationType.Chord;

        /// <summary>
        /// (en) Whether to clamp ends (default: true)
        /// (ja) 端点を固定するかどうか（デフォルト: true）
        /// </summary>
        public bool ClampEnds { get; set; } = true;
    }
}
