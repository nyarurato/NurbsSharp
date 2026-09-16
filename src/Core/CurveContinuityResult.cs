namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Result of continuity evaluation between two curves.
    /// (ja) 2つの曲線間の連続性評価結果。
    /// </summary>
    /// <remarks>
    /// <para>
    /// (en) A NaN metric means that the value was not evaluated or is mathematically undefined;
    /// it does not mean zero or a failed tolerance comparison. Tangent metrics are NaN when
    /// position continuity fails or either first derivative is the zero vector. Curvature metrics
    /// remain NaN when curvature comparison is not reached. CurvatureAngle is also NaN when exactly
    /// one curvature vector is zero because no angle can be defined.
    /// </para>
    /// <para>
    /// (ja) NaNのmetricは、その値が未評価または数学的に未定義であることを表し、ゼロや
    /// tolerance比較の不合格を意味しません。位置連続性が成立しない場合、またはいずれかの
    /// 一階微分がゼロベクトルの場合、接線metricはNaNになります。曲率比較へ到達しない場合、
    /// 曲率metricはNaNのままです。また片方の曲率ベクトルだけがゼロの場合は角度を定義できないため、
    /// CurvatureAngleはNaNになります。
    /// </para>
    /// <para>
    /// (en) In particular, C0 with a finite TangentAngle means that both tangents were defined but
    /// did not satisfy G1. C0 with a NaN TangentAngle means that position continuity was established,
    /// but higher continuity could not be evaluated because a tangent was degenerate. Use
    /// <see cref="System.Double.IsNaN(System.Double)"/> when testing metric availability.
    /// </para>
    /// <para>
    /// (ja) 特に、有限のTangentAngleを持つC0は両方の接線を評価できたもののG1を満たさなかった
    /// ことを表します。NaNのTangentAngleを持つC0は位置連続性までは確認できたものの、接線が退化して
    /// higher continuityを評価できなかったことを表します。metricが利用可能かどうかは
    /// <see cref="System.Double.IsNaN(System.Double)"/>で確認してください。
    /// </para>
    /// </remarks>
    public class CurveContinuityResult
    {
        /// <summary>
        /// (en) The highest level of continuity achieved.
        /// (ja) 達成された最高レベルの連続性。
        /// </summary>
        public ContinuityType Continuity { get; set; }

        /// <summary>
        /// (en) Position gap magnitude (for C0 check).
        /// (ja) 位置ギャップの大きさ（C0チェック用）。
        /// </summary>
        public double PositionGap { get; set; }

        /// <summary>
        /// (en) Tangent deviation angle in radians (for G1/C1 check), or NaN when not evaluated or undefined.
        /// (ja) 接線偏差角度（ラジアン、G1/C1チェック用）。未評価または未定義の場合はNaN。
        /// </summary>
        public double TangentAngle { get; set; }

        /// <summary>
        /// (en) Tangent magnitude ratio (first derivative ratio for C1 check), or NaN when not evaluated or undefined.
        /// (ja) 接線大きさ比率（C1チェック用の1階微分比率）。未評価または未定義の場合はNaN。
        /// </summary>
        public double TangentRatio { get; set; }

        /// <summary>
        /// (en) Curvature deviation angle in radians (for G2/C2 check), or NaN when not evaluated or undefined.
        /// (ja) 曲率偏差角度（ラジアン、G2/C2チェック用）。未評価または未定義の場合はNaN。
        /// </summary>
        public double CurvatureAngle { get; set; }

        /// <summary>
        /// (en) Curvature vector magnitude ratio (for G2 check), or NaN when not evaluated or undefined.
        /// (ja) 曲率ベクトルの大きさ比率（G2チェック用）。未評価または未定義の場合はNaN。
        /// </summary>
        public double CurvatureRatio { get; set; }

        /// <summary>
        /// (en) Whether the tangent parameter directions are opposite at the connection.
        /// (ja) 接続点で接線のパラメータ方向が反対向きか。
        /// </summary>
        public bool IsReversed { get; set; }

        /// <summary>
        /// (en) Initialize curve continuity result with default values.
        /// (ja) デフォルト値で曲線連続性評価結果を初期化します。
        /// </summary>
        public CurveContinuityResult()
        {
            Continuity = ContinuityType.None;
            PositionGap = double.MaxValue;
            TangentAngle = double.NaN;
            TangentRatio = double.NaN;
            CurvatureAngle = double.NaN;
            CurvatureRatio = double.NaN;
            IsReversed = false;
        }

        /// <summary>
        /// (en) Get string representation of curve continuity result.
        /// (ja) 曲線連続性評価結果の文字列表現を取得します。
        /// </summary>
        public override string ToString()
        {
            return $"Continuity: {Continuity}, Gap: {PositionGap:F6}, TangentAngle: {TangentAngle:F4} rad, Reversed: {IsReversed}";
        }
    }
}
