using System;
using NurbsSharp.Core;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Provides ray-box intersection tests
    /// (ja) Ray-Box交差判定を提供
    /// </summary>
    public static class RayBoxIntersector
    {
        /// <summary>
        /// (en) Tolerance for considering a ray direction component as zero (parallel to slab)
        /// (ja) レイの方向成分をゼロ（スラブに平行）とみなす許容誤差
        /// </summary>
        private const double DirectionTolerance = 1e-10;

        /// <summary>
        /// (en) Test if a ray intersects an axis-aligned bounding box using the slab method
        /// (ja) スラブ法を使用してレイが軸平行境界ボックスと交差するか判定
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="box">Bounding box to test against</param>
        /// <returns>True if the ray intersects the box</returns>
        public static bool Intersects(Ray ray, BoundingBox box)
        {
            return Intersects(ray, box, out _, out _);
        }

        /// <summary>
        /// (en) Test if a ray intersects an axis-aligned bounding box and return intersection parameters
        /// (ja) レイが軸平行境界ボックスと交差するか判定し、交差パラメータを返す
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="box">Bounding box to test against</param>
        /// <param name="tMin">Parameter of near intersection point (if intersects)</param>
        /// <param name="tMax">Parameter of far intersection point (if intersects)</param>
        /// <returns>True if the ray intersects the box</returns>
        /// <remarks>
        /// (en) Uses the slab method (Kay and Kajiya).
        ///      The method works by treating the box as the intersection of three pairs of parallel planes (slabs).
        ///      tMin and tMax represent the parameters along the ray where it enters and exits the box.
        ///      If tMax &lt; 0, the box is behind the ray origin.
        ///      If tMin &gt; tMax, the ray misses the box.
        /// (ja) スラブ法（Kay and Kajiya）を使用。
        ///      ボックスを3対の平行平面（スラブ）の交差として扱います。
        ///      tMinとtMaxはレイがボックスに入る/出る位置のパラメータを表します。
        ///      tMax &lt; 0の場合、ボックスはレイの原点の後ろにあります。
        ///      tMin &gt; tMaxの場合、レイはボックスを外れています。
        /// </remarks>
        public static bool Intersects(Ray ray, BoundingBox box, out double tMin, out double tMax)
        {
            tMin = double.NegativeInfinity;
            tMax = double.PositiveInfinity;

            // Check each axis (X, Y, Z)
            // For each axis, compute the intersection parameters with the two slabs

            // X axis
            if (Math.Abs(ray.Direction.X) < DirectionTolerance)
            {
                // Ray is parallel to the slab, check if origin is within the slab
                if (ray.Origin.X < box.Min.X || ray.Origin.X > box.Max.X)
                {
                    return false;
                }
            }
            else
            {
                double invD = 1.0 / ray.Direction.X;
                double t1 = (box.Min.X - ray.Origin.X) * invD;
                double t2 = (box.Max.X - ray.Origin.X) * invD;

                if (t1 > t2)
                {
                    // Swap to ensure t1 <= t2
                    (t1, t2) = (t2, t1);
                }

                tMin = Math.Max(tMin, t1);
                tMax = Math.Min(tMax, t2);

                if (tMin > tMax)
                {
                    return false;
                }
            }

            // Y axis
            if (Math.Abs(ray.Direction.Y) < DirectionTolerance)
            {
                if (ray.Origin.Y < box.Min.Y || ray.Origin.Y > box.Max.Y)
                {
                    return false;
                }
            }
            else
            {
                double invD = 1.0 / ray.Direction.Y;
                double t1 = (box.Min.Y - ray.Origin.Y) * invD;
                double t2 = (box.Max.Y - ray.Origin.Y) * invD;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                }

                tMin = Math.Max(tMin, t1);
                tMax = Math.Min(tMax, t2);

                if (tMin > tMax)
                {
                    return false;
                }
            }

            // Z axis
            if (Math.Abs(ray.Direction.Z) < DirectionTolerance)
            {
                if (ray.Origin.Z < box.Min.Z || ray.Origin.Z > box.Max.Z)
                {
                    return false;
                }
            }
            else
            {
                double invD = 1.0 / ray.Direction.Z;
                double t1 = (box.Min.Z - ray.Origin.Z) * invD;
                double t2 = (box.Max.Z - ray.Origin.Z) * invD;

                if (t1 > t2)
                {
                    (t1, t2) = (t2, t1);
                }

                tMin = Math.Max(tMin, t1);
                tMax = Math.Min(tMax, t2);

                if (tMin > tMax)
                {
                    return false;
                }
            }

            // If tMax < 0, the box is behind the ray origin
            if (tMax < 0)
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// (en) Get the nearest intersection point between a ray and a bounding box
        /// (ja) レイと境界ボックスの最近接交点を取得
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="box">Bounding box to test against</param>
        /// <param name="intersectionPoint">Intersection point (if intersects)</param>
        /// <returns>True if the ray intersects the box</returns>
        public static bool GetIntersectionPoint(Ray ray, BoundingBox box, out Vector3Double intersectionPoint)
        {
            if (Intersects(ray, box, out double tMin, out double tMax))
            {
                // Use tMin for the nearest intersection
                // If tMin < 0, the ray origin is inside the box, use tMax instead
                double t = tMin >= 0 ? tMin : tMax;
                intersectionPoint = ray.PointAt(t);
                return true;
            }

            intersectionPoint = Vector3Double.Zero;
            return false;
        }

        /// <summary>
        /// (en) Test if a ray segment intersects a bounding box
        /// (ja) レイセグメントが境界ボックスと交差するか判定
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="box">Bounding box to test against</param>
        /// <param name="maxDistance">Maximum distance along the ray to check</param>
        /// <returns>True if the ray segment intersects the box</returns>
        public static bool IntersectsSegment(Ray ray, BoundingBox box, double maxDistance)
        {
            if (Intersects(ray, box, out double tMin, out double tMax))
            {
                // Check if any part of the intersection is within [0, maxDistance]
                return tMax >= 0 && tMin <= maxDistance;
            }

            return false;
        }
    }
}
