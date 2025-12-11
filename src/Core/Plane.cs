using System;

namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Represents a plane in 3D space defined by the equation: Normal · (Point - Origin) = 0
    /// or in general form: Normal.X * x + Normal.Y * y + Normal.Z * z + Distance = 0
    /// (ja) 3D空間における平面を表す。方程式: Normal · (Point - Origin) = 0
    /// または一般形式: Normal.X * x + Normal.Y * y + Normal.Z * z + Distance = 0
    /// </summary>
    public readonly struct Plane : IEquatable<Plane>
    {
        /// <summary>
        /// (en) Normal vector of the plane (normalized)
        /// (ja) 平面の法線ベクトル（正規化済み）
        /// </summary>
        public Vector3Double Normal { get; }

        /// <summary>
        /// (en) Signed distance from the origin to the plane along the normal
        /// (ja) 原点から平面への符号付き距離（法線方向）
        /// In the plane equation ax + by + cz + d = 0, this is the 'd' value
        /// </summary>
        public double Distance { get; }

        /// <summary>
        /// (en) Indicates whether the normal vector is normalized
        /// (ja) 法線ベクトルが正規化されているかを示す
        /// </summary>
        public bool IsNormalized { get; }

        /// <summary>
        /// (en) Constructor with normal vector and distance from origin
        /// (ja) 法線ベクトルと原点からの距離で構築
        /// </summary>
        /// <param name="normal">Normal vector of the plane</param>
        /// <param name="distance">Signed distance from origin</param>
        /// <param name="normalize">Whether to normalize the normal vector (default: true)</param>
        /// <exception cref="ArgumentException">Thrown when normal is zero vector</exception>
        public Plane(Vector3Double normal, double distance, bool normalize = true)
        {
            if (normal.magnitude == 0)
                throw new ArgumentException("Normal vector cannot be zero.", nameof(normal));

            if (normalize)
            {
                double mag = normal.magnitude;
                Normal = normal.normalized;
                Distance = distance / mag; // Normalize distance as well
                IsNormalized = true;
            }
            else
            {
                Normal = normal;
                Distance = distance;
                IsNormalized = false;
            }
        }

        /// <summary>
        /// (en) Constructor with normal vector and a point on the plane
        /// (ja) 法線ベクトルと平面上の点で構築
        /// </summary>
        /// <param name="normal">Normal vector of the plane</param>
        /// <param name="pointOnPlane">A point that lies on the plane</param>
        /// <param name="normalize">Whether to normalize the normal vector (default: true)</param>
        /// <exception cref="ArgumentException">Thrown when normal is zero vector</exception>
        public Plane(Vector3Double normal, Vector3Double pointOnPlane, bool normalize = true)
        {
            if (normal.magnitude == 0)
                throw new ArgumentException("Normal vector cannot be zero.", nameof(normal));

            if (normalize)
            {
                Normal = normal.normalized;
                IsNormalized = true;
            }
            else
            {
                Normal = normal;
                IsNormalized = false;
            }

            // Calculate distance: d = -Normal · PointOnPlane
            Distance = -(Normal.X * pointOnPlane.X + Normal.Y * pointOnPlane.Y + Normal.Z * pointOnPlane.Z);
        }

        /// <summary>
        /// (en) Constructor from three non-collinear points on the plane
        /// (ja) 平面上の3つの非共線点から構築
        /// </summary>
        /// <param name="p1">First point on the plane</param>
        /// <param name="p2">Second point on the plane</param>
        /// <param name="p3">Third point on the plane</param>
        /// <exception cref="ArgumentException">Thrown when points are collinear</exception>
        public Plane(Vector3Double p1, Vector3Double p2, Vector3Double p3)
        {
            // Calculate two edge vectors
            Vector3Double v1 = p2 - p1;
            Vector3Double v2 = p3 - p1;

            // Calculate normal as cross product
            Vector3Double cross = new Vector3Double(
                v1.Y * v2.Z - v1.Z * v2.Y,
                v1.Z * v2.X - v1.X * v2.Z,
                v1.X * v2.Y - v1.Y * v2.X
            );

            if (cross.magnitude == 0)
                throw new ArgumentException("Points must not be collinear.");

            Normal = cross.normalized;
            IsNormalized = true;

            // Calculate distance using p1
            Distance = -(Normal.X * p1.X + Normal.Y * p1.Y + Normal.Z * p1.Z);
        }

        /// <summary>
        /// (en) Calculate the signed distance from a point to the plane
        /// (ja) 点から平面への符号付き距離を計算
        /// Positive if point is on the side of the normal, negative otherwise
        /// </summary>
        /// <param name="point">Point to measure distance from</param>
        /// <returns>Signed distance</returns>
        public double SignedDistanceTo(Vector3Double point)
        {
            double raw = Normal.X * point.X + Normal.Y * point.Y + Normal.Z * point.Z + Distance;
            // Return true Euclidean signed distance. If normal is not normalized, divide by magnitude.
            return IsNormalized ? raw : raw / Normal.magnitude;
        }

        /// <summary>
        /// (en) Calculate the absolute distance from a point to the plane
        /// (ja) 点から平面への絶対距離を計算
        /// </summary>
        /// <param name="point">Point to measure distance from</param>
        /// <returns>Absolute distance</returns>
        public double DistanceTo(Vector3Double point)
        {
            return Math.Abs(SignedDistanceTo(point));
        }

        /// <summary>
        /// (en) Get the closest point on the plane to the given point
        /// (ja) 与えられた点に最も近い平面上の点を取得
        /// </summary>
        /// <param name="point">Point to project onto the plane</param>
        /// <returns>Closest point on the plane</returns>
        public Vector3Double ClosestPoint(Vector3Double point)
        {
            // Compute using raw dot product and adjust for non-normalized normals.
            double raw = Normal.X * point.X + Normal.Y * point.Y + Normal.Z * point.Z + Distance;
            if (IsNormalized)
            {
                return point - Normal * raw;
            }

            // For non-normalized normals, closest point is point - Normal * (raw / |N|^2)
            double normSq = Normal.magnitude * Normal.magnitude;
            return point - Normal * (raw / normSq);
        }

        /// <summary>
        /// (en) Project a point onto the plane
        /// (ja) 点を平面に投影する
        /// </summary>
        /// <param name="point">Point to project</param>
        /// <returns>Projected point on the plane</returns>
        public Vector3Double ProjectPoint(Vector3Double point)
        {
            return ClosestPoint(point);
        }

        /// <summary>
        /// (en) Check if a point lies on the plane within tolerance
        /// (ja) 点が許容誤差内で平面上にあるかをチェック
        /// </summary>
        /// <param name="point">Point to test</param>
        /// <param name="tolerance">Tolerance for distance comparison</param>
        /// <returns>True if point is on the plane</returns>
        public bool Contains(Vector3Double point, double tolerance = 1e-9)
        {
            return Math.Abs(SignedDistanceTo(point)) <= tolerance;
        }

        /// <summary>
        /// (en) Flip the plane (reverse normal direction)
        /// (ja) 平面を反転（法線方向を逆転）
        /// </summary>
        /// <returns>Flipped plane</returns>
        public Plane Flip()
        {
            return new Plane(-Normal, -Distance, normalize: IsNormalized);
        }

        /// <summary>
        /// (en) Get a normalized version of this plane
        /// (ja) この平面の正規化版を取得
        /// </summary>
        /// <returns>Plane with normalized normal</returns>
        public Plane Normalized()
        {
            if (IsNormalized)
                return this;

            return new Plane(Normal, Distance, normalize: true);
        }

        /// <summary>
        /// (en) Translate the plane by an offset
        /// (ja) 平面をオフセット分移動
        /// </summary>
        /// <param name="offset">Translation vector</param>
        /// <returns>Translated plane</returns>
        public Plane Translate(Vector3Double offset)
        {
            // When translating, only the distance changes
            double newDistance = Distance - (Normal.X * offset.X + Normal.Y * offset.Y + Normal.Z * offset.Z);
            return new Plane(Normal, newDistance, normalize: IsNormalized);
        }

        /// <summary>
        /// (en) Intersect the plane with a ray
        /// (ja) 平面とレイの交差判定
        /// </summary>
        /// <param name="ray">Ray to intersect with</param>
        /// <param name="t">Parameter value along the ray (if intersection exists)</param>
        /// <returns>True if intersection exists</returns>
        public bool Raycast(Ray ray, out double t)
        {
            double denominator = Normal.X * ray.Direction.X + 
                                Normal.Y * ray.Direction.Y + 
                                Normal.Z * ray.Direction.Z;

            // Ray is parallel to plane
            if (Math.Abs(denominator) < 1e-10)
            {
                t = 0;
                return false;
            }

            double numerator = -(Normal.X * ray.Origin.X + 
                                Normal.Y * ray.Origin.Y + 
                                Normal.Z * ray.Origin.Z + 
                                Distance);

            t = numerator / denominator;
            return t >= 0; // Only return true for forward intersections
        }

        /// <summary>
        /// (en) Get the intersection point between the plane and a ray
        /// (ja) 平面とレイの交点を取得
        /// </summary>
        /// <param name="ray">Ray to intersect with</param>
        /// <param name="intersectionPoint">Intersection point (if exists)</param>
        /// <returns>True if intersection exists</returns>
        public bool Intersect(Ray ray, out Vector3Double intersectionPoint)
        {
            if (Raycast(ray, out double t))
            {
                intersectionPoint = ray.PointAt(t);
                return true;
            }

            intersectionPoint = Vector3Double.Zero;
            return false;
        }

        /// <summary>
        /// Determines whether the specified plane is equal to the current plane.
        /// </summary>
        public bool Equals(Plane other)
        {
            return Normal == other.Normal && Distance == other.Distance;
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current plane.
        /// </summary>
        public override bool Equals(object? obj)
        {
            return obj is Plane other && Equals(other);
        }

        /// <summary>
        /// Returns the hash code for this plane.
        /// </summary>
        public override int GetHashCode()
        {
            return HashCode.Combine(Normal, Distance);
        }

        /// <summary>
        /// Determines whether two planes are equal.
        /// </summary>
        public static bool operator ==(Plane left, Plane right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Determines whether two planes are not equal.
        /// </summary>
        public static bool operator !=(Plane left, Plane right)
        {
            return !left.Equals(right);
        }

        /// <summary>
        /// Returns a string that represents the current plane.
        /// </summary>
        public override string ToString()
        {
            return $"Plane [Normal: {Normal}, Distance: {Distance}]";
        }

        /// <summary>
        /// (en) Standard XY plane (Z = 0)
        /// (ja) 標準XY平面 (Z = 0)
        /// </summary>
        public static Plane XY => new Plane(new Vector3Double(0, 0, 1), 0, normalize: true);

        /// <summary>
        /// (en) Standard YZ plane (X = 0)
        /// (ja) 標準YZ平面 (X = 0)
        /// </summary>
        public static Plane YZ => new Plane(new Vector3Double(1, 0, 0), 0, normalize: true);

        /// <summary>
        /// (en) Standard XZ plane (Y = 0)
        /// (ja) 標準XZ平面 (Y = 0)
        /// </summary>
        public static Plane XZ => new Plane(new Vector3Double(0, 1, 0), 0, normalize: true);
    }
}
