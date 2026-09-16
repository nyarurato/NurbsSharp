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
}
