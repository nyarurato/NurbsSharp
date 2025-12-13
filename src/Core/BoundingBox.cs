using System;
using System.Collections.Generic;
using System.Linq;

namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Axis-Aligned Bounding Box (AABB)
    /// (ja) 軸平行境界ボックス
    /// </summary>
    public readonly struct BoundingBox : IEquatable<BoundingBox>
    {
        /// <summary>
        /// (en) Minimum corner of the bounding box
        /// (ja) 境界ボックスの最小コーナー
        /// </summary>
        public Vector3Double Min { get; }

        /// <summary>
        /// (en) Maximum corner of the bounding box
        /// (ja) 境界ボックスの最大コーナー
        /// </summary>
        public Vector3Double Max { get; }

        /// <summary>
        /// (en) Center point of the bounding box
        /// (ja) 境界ボックスの中心点
        /// </summary>
        public Vector3Double Center => (Min + Max) * 0.5;

        /// <summary>
        /// (en) Size of the bounding box in each dimension
        /// (ja) 各次元での境界ボックスのサイズ
        /// </summary>
        public Vector3Double Size => Max - Min;

        /// <summary>
        /// (en) Volume of the bounding box
        /// (ja) 境界ボックスの体積
        /// </summary>
        public double Volume
        {
            get
            {
                var size = Size;
                return size.X * size.Y * size.Z;
            }
        }

        /// <summary>
        /// (en) Surface area of the bounding box
        /// (ja) 境界ボックスの表面積
        /// </summary>
        public double SurfaceArea
        {
            get
            {
                var size = Size;
                return 2.0 * (size.X * size.Y + size.Y * size.Z + size.Z * size.X);
            }
        }

        /// <summary>
        /// (en) Check if the bounding box is valid (Min &lt;= Max in all dimensions)
        /// (ja) 境界ボックスが有効か確認（すべての次元でMin &lt;= Max）
        /// </summary>
        public bool IsValid => Min.X <= Max.X && Min.Y <= Max.Y && Min.Z <= Max.Z;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="min">Minimum corner</param>
        /// <param name="max">Maximum corner</param>
        /// <exception cref="ArgumentException"></exception>
        public BoundingBox(Vector3Double min, Vector3Double max)
        {
            Min = min;
            Max = max;
            if (!IsValid)
                throw new ArgumentException("Invalid BoundingBox: Min must be less than or equal to Max in all dimensions.");
        }

        /// <summary>
        /// (en) Create a bounding box from a collection of points
        /// (ja) 点のコレクションから境界ボックスを作成
        /// </summary>
        /// <param name="points">Collection of points</param>
        /// <returns>Bounding box containing all points</returns>
        /// <exception cref="ArgumentException">Thrown when points collection is empty</exception>
        public static BoundingBox FromPoints(IEnumerable<Vector3Double> points)
        {
            if (points == null || !points.Any())
                throw new ArgumentException("Points collection cannot be null or empty.", nameof(points));

            double minX = double.PositiveInfinity;
            double minY = double.PositiveInfinity;
            double minZ = double.PositiveInfinity;
            double maxX = double.NegativeInfinity;
            double maxY = double.NegativeInfinity;
            double maxZ = double.NegativeInfinity;

            foreach (var point in points)
            {
                if (point.X < minX) minX = point.X;
                if (point.Y < minY) minY = point.Y;
                if (point.Z < minZ) minZ = point.Z;
                if (point.X > maxX) maxX = point.X;
                if (point.Y > maxY) maxY = point.Y;
                if (point.Z > maxZ) maxZ = point.Z;
            }

            return new BoundingBox(
                new Vector3Double(minX, minY, minZ),
                new Vector3Double(maxX, maxY, maxZ)
            );
        }

        /// <summary>
        /// (en) Create a bounding box from control points
        /// (ja) 制御点から境界ボックスを作成
        /// </summary>
        /// <param name="controlPoints">Array of control points</param>
        /// <returns>Bounding box containing all control points</returns>
        /// <remarks>
        /// (en) This method creates a bounding box by control points, it may not fit the actual shape.
        /// (ja) 制御点からボックスを作成するため、実際の形状にフィットしない場合があります。
        /// </remarks>
        public static BoundingBox FromControlPoints(ControlPoint[] controlPoints)
        {
            Guard.ThrowIfNull(controlPoints, nameof(controlPoints));
            if (controlPoints.Length == 0)
                throw new ArgumentException("Control points array cannot be empty.", nameof(controlPoints));

            return FromPoints(controlPoints.Select(cp => cp.Position));
        }

        /// <summary>
        /// (en) Create a bounding box from 2D control points array (for surfaces)
        /// (ja) 2次元制御点配列から境界ボックスを作成（サーフェス用）
        /// </summary>
        /// <param name="controlPoints">2D array of control points</param>
        /// <returns>Bounding box containing all control points</returns>
        /// <remarks>
        /// (en) This method creates a bounding box by control points, it may not fit the actual shape.
        /// (ja) 制御点からボックスを作成するため、実際の形状にフィットしない場合があります。
        /// </remarks>
        public static BoundingBox FromControlPoints(ControlPoint[][] controlPoints)
        {
            Guard.ThrowIfNull(controlPoints, nameof(controlPoints));
            if (controlPoints.Length == 0)
                throw new ArgumentException("Control points array cannot be empty.", nameof(controlPoints));

            var allPoints = controlPoints.SelectMany(row => row.Select(cp => cp.Position));
            return FromPoints(allPoints);
        }

        /// <summary>
        /// (en) Create a bounding box from 3D control points array (for volumes)
        /// (ja) 3次元制御点配列から境界ボックスを作成（ボリューム用）
        /// </summary>
        /// <param name="controlPoints"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public static BoundingBox FromControlPoints(ControlPoint[][][] controlPoints)
        {
            Guard.ThrowIfNull(controlPoints, nameof(controlPoints));
            if (controlPoints.Length == 0)
                throw new ArgumentException("Control points array cannot be empty.", nameof(controlPoints));

            var allPoints = controlPoints.SelectMany(
                matrix => matrix.SelectMany(
                    row => row.Select(cp => cp.Position)
                )
            );
            return FromPoints(allPoints);
        }

        /// <summary>
        /// (en) Check if a point is contained within the bounding box
        /// (ja) 点が境界ボックス内に含まれるか確認
        /// </summary>
        /// <param name="point">Point to check</param>
        /// <returns>True if point is inside or on the boundary</returns>
        public bool Contains(Vector3Double point)
        {
            return point.X >= Min.X && point.X <= Max.X &&
                   point.Y >= Min.Y && point.Y <= Max.Y &&
                   point.Z >= Min.Z && point.Z <= Max.Z;
        }

        /// <summary>
        /// (en) Check if this bounding box intersects with another
        /// (ja) この境界ボックスが他の境界ボックスと交差するか確認
        /// </summary>
        /// <param name="other">Other bounding box</param>
        /// <returns>True if the boxes intersect</returns>
        public bool Intersects(BoundingBox other)
        {
            return Min.X <= other.Max.X && Max.X >= other.Min.X &&
                   Min.Y <= other.Max.Y && Max.Y >= other.Min.Y &&
                   Min.Z <= other.Max.Z && Max.Z >= other.Min.Z;
        }

        /// <summary>
        /// (en) Compute the union of this bounding box with another
        /// (ja) この境界ボックスと他の境界ボックスの和を計算
        /// </summary>
        /// <param name="other">Other bounding box</param>
        /// <returns>Bounding box containing both boxes</returns>
        public BoundingBox Union(BoundingBox other)
        {
            return new BoundingBox(
                new Vector3Double(
                    Math.Min(Min.X, other.Min.X),
                    Math.Min(Min.Y, other.Min.Y),
                    Math.Min(Min.Z, other.Min.Z)
                ),
                new Vector3Double(
                    Math.Max(Max.X, other.Max.X),
                    Math.Max(Max.Y, other.Max.Y),
                    Math.Max(Max.Z, other.Max.Z)
                )
            );
        }

        /// <summary>
        /// (en) Expand the bounding box by a margin in all directions
        /// (ja) すべての方向に余白を追加して境界ボックスを拡大
        /// </summary>
        /// <param name="margin">Margin to add</param>
        /// <returns>Expanded bounding box</returns>
        public BoundingBox Expand(double margin)
        {
            var offset = new Vector3Double(margin, margin, margin);
            return new BoundingBox(Min - offset, Max + offset);
        }

        /// <summary>
        /// (en) Subdivide the bounding box into 8 octants
        /// (ja) 境界ボックスを8つのオクタントに分割
        /// </summary>
        /// <returns>Array of 8 child bounding boxes</returns>
        public BoundingBox[] Subdivide()
        {
            var center = Center;
            var boxes = new BoundingBox[8];

            // Order: (x-, y-, z-), (x+, y-, z-), (x-, y+, z-), (x+, y+, z-),
            //        (x-, y-, z+), (x+, y-, z+), (x-, y+, z+), (x+, y+, z+)
            boxes[0] = new BoundingBox(Min, center);
            boxes[1] = new BoundingBox(new Vector3Double(center.X, Min.Y, Min.Z), new Vector3Double(Max.X, center.Y, center.Z));
            boxes[2] = new BoundingBox(new Vector3Double(Min.X, center.Y, Min.Z), new Vector3Double(center.X, Max.Y, center.Z));
            boxes[3] = new BoundingBox(new Vector3Double(center.X, center.Y, Min.Z), new Vector3Double(Max.X, Max.Y, center.Z));
            boxes[4] = new BoundingBox(new Vector3Double(Min.X, Min.Y, center.Z), new Vector3Double(center.X, center.Y, Max.Z));
            boxes[5] = new BoundingBox(new Vector3Double(center.X, Min.Y, center.Z), new Vector3Double(Max.X, center.Y, Max.Z));
            boxes[6] = new BoundingBox(new Vector3Double(Min.X, center.Y, center.Z), new Vector3Double(center.X, Max.Y, Max.Z));
            boxes[7] = new BoundingBox(center, Max);

            return boxes;
        }

        /// <summary>
        /// (en) Get the closest point on the bounding box to a given point
        /// (ja) 与えられた点に最も近い境界ボックス上の点を取得
        /// </summary>
        /// <param name="point">Input point</param>
        /// <returns>Closest point on or inside the bounding box</returns>
        public Vector3Double ClosestPoint(Vector3Double point)
        {
            return new Vector3Double(
                Math.Clamp(point.X, Min.X, Max.X),
                Math.Clamp(point.Y, Min.Y, Max.Y),
                Math.Clamp(point.Z, Min.Z, Max.Z)
            );
        }

        /// <summary>
        /// (en) Calculate the distance from a point to the bounding box
        /// (ja) 点から境界ボックスまでの距離を計算
        /// </summary>
        /// <param name="point">Input point</param>
        /// <returns>Distance (0 if point is inside)</returns>
        public double DistanceTo(Vector3Double point)
        {
            var closest = ClosestPoint(point);
            return point.DistanceTo(closest);
        }

        /// <summary>
        /// Determines whether the specified bounding box is equal to the current bounding box.
        /// </summary>
        public bool Equals(BoundingBox other)
        {
            return Min == other.Min && Max == other.Max;
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current bounding box.
        /// </summary>
        public override bool Equals(object? obj)
        {
            return obj is BoundingBox other && Equals(other);
        }

        /// <summary>
        /// Returns the hash code for this bounding box.
        /// </summary>
        public override int GetHashCode()
        {
            return HashCode.Combine(Min, Max);
        }

        /// <summary>
        /// Determines whether two bounding boxes are equal.
        /// </summary>
        public static bool operator ==(BoundingBox left, BoundingBox right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Determines whether two bounding boxes are not equal.
        /// </summary>
        public static bool operator !=(BoundingBox left, BoundingBox right)
        {
            return !left.Equals(right);
        }

        /// <summary>
        /// Returns a string that represents the current bounding box.
        /// </summary>
        public override string ToString()
        {
            return $"BoundingBox [Min: {Min}, Max: {Max}]";
        }
    }
}
