using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;

namespace NurbsSharp.Geometry
{
    /// <summary>
    /// Interface for geometric entities
    /// </summary>
    public interface IGeometry
    {
        /// <summary>
        /// (en) Gets the axis-aligned bounding box of this geometry
        /// (ja) この形状の軸平行境界ボックスを取得
        /// </summary>
        BoundingBox BoundingBox { get; }
    }
}
