namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Defines types of continuity between geometric entities (curves/surfaces).
    /// (ja) 幾何要素（曲線/曲面）間の連続性タイプを定義します。
    /// </summary>
    public enum ContinuityType
    {
        /// <summary>
        /// (en) No continuity - entities are disconnected.
        /// (ja) 連続性なし - 要素は非接続。
        /// </summary>
        None = 0,

        /// <summary>
        /// (en) C0 continuity - Position continuity (endpoints match).
        /// (ja) C0連続 - 位置連続性（端点が一致）。
        /// </summary>
        C0 = 1,

        /// <summary>
        /// (en) G1 continuity - Geometric tangent continuity (tangent vectors have the same direction, magnitudes may differ).
        /// (ja) G1連続 - 幾何学的接線連続性（接線ベクトルが同方向、大きさは異なってもよい）。
        /// </summary>
        G1 = 2,

        /// <summary>
        /// (en) C1 continuity - Parametric tangent continuity (first derivatives match exactly).
        /// (ja) C1連続 - パラメトリック接線連続性（1階微分が完全一致）。
        /// </summary>
        C1 = 3,

        /// <summary>
        /// (en) G2 continuity - Geometric curvature continuity (curvature vectors match within tolerance).
        /// (ja) G2連続 - 幾何学的曲率連続性（曲率ベクトルが許容誤差内で一致）。
        /// </summary>
        G2 = 4,

        /// <summary>
        /// (en) C2 continuity - Parametric curvature continuity (second derivatives match exactly).
        /// (ja) C2連続 - パラメトリック曲率連続性（2階微分が完全一致）。
        /// </summary>
        C2 = 5
    }

    /// <summary>
    /// (en) Result of continuity evaluation between two geometric entities.
    /// (ja) 2つの幾何要素間の連続性評価結果。
    /// </summary>
    public class ContinuityResult
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
        /// (en) Initialize continuity result with default values.
        /// (ja) デフォルト値で連続性評価結果を初期化します。
        /// </summary>
        public ContinuityResult()
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
        /// (en) Get string representation of continuity result.
        /// (ja) 連続性評価結果の文字列表現を取得します。
        /// </summary>
        public override string ToString()
        {
            return $"Continuity: {Continuity}, Gap: {PositionGap:F6}, TangentAngle: {TangentAngle:F4} rad, Reversed: {IsReversed}";
        }
    }
}
