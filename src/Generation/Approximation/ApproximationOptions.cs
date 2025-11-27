using NurbsSharp.Generation.Interpolation;

namespace NurbsSharp.Generation.Approximation
{
    /// <summary>
    /// (en) Options for NURBS approximation
    /// (ja) NURBS近似のオプション
    /// </summary>
    public class ApproximationOptions
    {
        /// <summary>
        /// (en) Parameterization type (default: Chord)
        /// (ja) パラメータ化タイプ（デフォルト: 弦長）
        /// </summary>
        public ParameterizationType ParameterizationType { get; set; } = ParameterizationType.Chord;

        /// <summary>
        /// (en) Whether to clamp ends - fixes first and last control points to data points (default: true)
        /// (ja) 端点を固定するかどうか - 最初と最後の制御点をデータ点に固定 (デフォルト: true)
        /// </summary>
        public bool ClampEnds { get; set; } = true;
    }
}
