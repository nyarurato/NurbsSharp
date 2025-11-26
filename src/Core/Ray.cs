using System;

namespace NurbsSharp.Core
{
    /// <summary>
    /// (en) Represents a ray with an origin point and a direction vector
    /// (ja) 原点と方向ベクトルを持つレイを表す
    /// </summary>
    public readonly struct Ray : IEquatable<Ray>
    {
        /// <summary>
        /// (en) Origin point of the ray
        /// (ja) レイの原点
        /// </summary>
        public Vector3Double Origin { get; }

        /// <summary>
        /// (en) Direction vector of the ray (normalized)
        /// (ja) レイの方向ベクトル（正規化済み）
        /// </summary>
        public Vector3Double Direction { get; }

        /// <summary>
        /// (en) Indicates whether the direction vector is normalized
        /// (ja) 方向ベクトルが正規化されているかを示す
        /// </summary>
        public bool IsNormalized { get; }

        /// <summary>
        /// Constructor with automatic normalization
        /// </summary>
        /// <param name="origin">Origin point of the ray</param>
        /// <param name="direction">Direction vector (will be normalized)</param>
        /// <param name="normalize">Whether to normalize the direction vector (default: true)</param>
        /// <exception cref="ArgumentException">Thrown when direction is zero vector</exception>
        public Ray(Vector3Double origin, Vector3Double direction, bool normalize = true)
        {
            Origin = origin;
            
            if (direction.magnitude == 0)
                throw new ArgumentException("Direction vector cannot be zero.", nameof(direction));

            if (normalize)
            {
                Direction = direction.normalized;
                IsNormalized = true;
            }
            else
            {
                Direction = direction;
                IsNormalized = false;
            }
        }

        /// <summary>
        /// (en) Get a point along the ray at parameter t
        /// (ja) パラメータtにおけるレイ上の点を取得
        /// </summary>
        /// <param name="t">Distance parameter along the ray (t >= 0)</param>
        /// <returns>Point at position Origin + t * Direction</returns>
        public Vector3Double PointAt(double t)
        {
            return Origin + Direction * t;
        }
        
        /// <summary>
        /// (en) Get a normalized version of this ray
        /// (ja) このレイの正規化版を取得
        /// </summary>
        /// <returns>Ray with normalized direction</returns>
        public Ray Normalized()
        {
            if (IsNormalized)
                return this;
            
            return new Ray(Origin, Direction, normalize: true);
        }

        /// <summary>
        /// (en) Transform the ray by translation
        /// (ja) レイを平行移動
        /// </summary>
        /// <param name="offset">Translation vector</param>
        /// <returns>Translated ray</returns>
        public Ray Translate(Vector3Double offset)
        {
            return new Ray(Origin + offset, Direction, normalize: false);
        }

        /// <summary>
        /// Determines whether the specified ray is equal to the current ray.
        /// </summary>
        public bool Equals(Ray other)
        {
            return Origin == other.Origin && Direction == other.Direction;
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current ray.
        /// </summary>
        public override bool Equals(object? obj)
        {
            return obj is Ray other && Equals(other);
        }

        /// <summary>
        /// Returns the hash code for this ray.
        /// </summary>
        public override int GetHashCode()
        {
            return HashCode.Combine(Origin, Direction);
        }

        /// <summary>
        /// Determines whether two rays are equal.
        /// </summary>
        public static bool operator ==(Ray left, Ray right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Determines whether two rays are not equal.
        /// </summary>
        public static bool operator !=(Ray left, Ray right)
        {
            return !left.Equals(right);
        }

        /// <summary>
        /// Returns a string that represents the current ray.
        /// </summary>
        public override string ToString()
        {
            return $"Ray [Origin: {Origin}, Direction: {Direction}]";
        }
    }
}
