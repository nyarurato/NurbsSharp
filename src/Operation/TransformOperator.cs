using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for affine transformations on NURBS geometries
    /// (ja) NURBSジオメトリのアフィン変換操作クラス
    /// </summary>
    public static class TransformOperator
    {
        #region Non-destructive transformations (return new instance)

        /// <summary>
        /// (en) Transform curve using transformation matrix (non-destructive)
        /// (ja) 変換行列を使用して曲線を変換（非破壊的）
        /// </summary>
        /// <param name="curve">Source curve</param>
        /// <param name="matrix">Transformation matrix</param>
        /// <returns>New transformed curve</returns>
        public static NurbsCurve Transform(NurbsCurve curve, Matrix4x4 matrix)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            ControlPoint[] newControlPoints = new ControlPoint[curve.ControlPoints.Length];
            for (int i = 0; i < curve.ControlPoints.Length; i++)
            {
                Vector3Double transformedPos = matrix.TransformPoint(curve.ControlPoints[i].Position);
                newControlPoints[i] = new ControlPoint(transformedPos, curve.ControlPoints[i].Weight);
            }

            return new NurbsCurve(curve.Degree, curve.KnotVector, newControlPoints);
        }

        /// <summary>
        /// (en) Transform surface using transformation matrix (non-destructive)
        /// (ja) 変換行列を使用してサーフェスを変換（非破壊的）
        /// </summary>
        /// <param name="surface">Source surface</param>
        /// <param name="matrix">Transformation matrix</param>
        /// <returns>New transformed surface</returns>
        public static NurbsSurface Transform(NurbsSurface surface, Matrix4x4 matrix)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            ControlPoint[][] newControlPoints = new ControlPoint[surface.ControlPoints.Length][];
            for (int i = 0; i < surface.ControlPoints.Length; i++)
            {
                newControlPoints[i] = new ControlPoint[surface.ControlPoints[i].Length];
                for (int j = 0; j < surface.ControlPoints[i].Length; j++)
                {
                    Vector3Double transformedPos = matrix.TransformPoint(surface.ControlPoints[i][j].Position);
                    newControlPoints[i][j] = new ControlPoint(transformedPos, surface.ControlPoints[i][j].Weight);
                }
            }

            return new NurbsSurface(surface.DegreeU, surface.DegreeV, 
                surface.KnotVectorU, surface.KnotVectorV, newControlPoints);
        }

        /// <summary>
        /// (en) Transform volume using transformation matrix (non-destructive)
        /// (ja) 変換行列を使用してボリュームを変換（非破壊的）
        /// </summary>
        /// <param name="volume">Source volume</param>
        /// <param name="matrix">Transformation matrix</param>
        /// <returns>New transformed volume</returns>
        public static NurbsVolume Transform(NurbsVolume volume, Matrix4x4 matrix)
        {
            Guard.ThrowIfNull(volume, nameof(volume));

            ControlPoint[][][] newControlPoints = new ControlPoint[volume.ControlPoints.Length][][];
            for (int i = 0; i < volume.ControlPoints.Length; i++)
            {
                newControlPoints[i] = new ControlPoint[volume.ControlPoints[i].Length][];
                for (int j = 0; j < volume.ControlPoints[i].Length; j++)
                {
                    newControlPoints[i][j] = new ControlPoint[volume.ControlPoints[i][j].Length];
                    for (int k = 0; k < volume.ControlPoints[i][j].Length; k++)
                    {
                        Vector3Double transformedPos = matrix.TransformPoint(volume.ControlPoints[i][j][k].Position);
                        newControlPoints[i][j][k] = new ControlPoint(transformedPos, volume.ControlPoints[i][j][k].Weight);
                    }
                }
            }

            return new NurbsVolume(volume.DegreeU, volume.DegreeV, volume.DegreeW,
                volume.KnotVectorU, volume.KnotVectorV, volume.KnotVectorW, newControlPoints);
        }

        /// <summary>
        /// (en) Translate curve (non-destructive)
        /// (ja) 曲線を平行移動（非破壊的）
        /// </summary>
        public static NurbsCurve Translate(NurbsCurve curve, Vector3Double delta)
        {
            return Transform(curve, Matrix4x4.CreateTranslation(delta));
        }

        /// <summary>
        /// (en) Translate curve (non-destructive)
        /// (ja) 曲線を平行移動（非破壊的）
        /// </summary>
        public static NurbsCurve Translate(NurbsCurve curve, double dx, double dy, double dz)
        {
            return Transform(curve, Matrix4x4.CreateTranslation(dx, dy, dz));
        }

        /// <summary>
        /// (en) Translate surface (non-destructive)
        /// (ja) サーフェスを平行移動（非破壊的）
        /// </summary>
        public static NurbsSurface Translate(NurbsSurface surface, Vector3Double delta)
        {
            return Transform(surface, Matrix4x4.CreateTranslation(delta));
        }

        /// <summary>
        /// (en) Translate surface (non-destructive)
        /// (ja) サーフェスを平行移動（非破壊的）
        /// </summary>
        public static NurbsSurface Translate(NurbsSurface surface, double dx, double dy, double dz)
        {
            return Transform(surface, Matrix4x4.CreateTranslation(dx, dy, dz));
        }

        /// <summary>
        /// (en) Translate volume (non-destructive)
        /// (ja) ボリュームを平行移動（非破壊的）
        /// </summary>
        public static NurbsVolume Translate(NurbsVolume volume, Vector3Double delta)
        {
            return Transform(volume, Matrix4x4.CreateTranslation(delta));
        }

        /// <summary>
        /// (en) Translate volume (non-destructive)
        /// (ja) ボリュームを平行移動（非破壊的）
        /// </summary>
        public static NurbsVolume Translate(NurbsVolume volume, double dx, double dy, double dz)
        {
            return Transform(volume, Matrix4x4.CreateTranslation(dx, dy, dz));
        }

        /// <summary>
        /// (en) Rotate curve around axis (non-destructive)
        /// (ja) 曲線を軸周りに回転（非破壊的）
        /// </summary>
        /// <param name="curve">Source curve</param>
        /// <param name="axis">Rotation axis</param>
        /// <param name="angle">Angle in radians</param>
        /// <param name="center">Center of rotation (default: origin)</param>
        public static NurbsCurve Rotate(NurbsCurve curve, Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateRotation(axis, angle, center.Value)
                : Matrix4x4.CreateRotation(axis, angle);
            return Transform(curve, matrix);
        }

        /// <summary>
        /// (en) Rotate surface around axis (non-destructive)
        /// (ja) サーフェスを軸周りに回転（非破壊的）
        /// </summary>
        /// <param name="surface">Source surface</param>
        /// <param name="axis">Rotation axis</param>
        /// <param name="angle">Angle in radians</param>
        /// <param name="center">Center of rotation (default: origin)</param>
        public static NurbsSurface Rotate(NurbsSurface surface, Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateRotation(axis, angle, center.Value)
                : Matrix4x4.CreateRotation(axis, angle);
            return Transform(surface, matrix);
        }

        /// <summary>
        /// (en) Rotate volume around axis (non-destructive)
        /// (ja) ボリュームを軸周りに回転（非破壊的）
        /// </summary>
        /// <param name="volume">Source volume</param>
        /// <param name="axis">Rotation axis</param>
        /// <param name="angle">Angle in radians</param>
        /// <param name="center">Center of rotation (default: origin)</param>
        public static NurbsVolume Rotate(NurbsVolume volume, Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateRotation(axis, angle, center.Value)
                : Matrix4x4.CreateRotation(axis, angle);
            return Transform(volume, matrix);
        }

        /// <summary>
        /// (en) Scale curve uniformly (non-destructive)
        /// (ja) 曲線を一様にスケール（非破壊的）
        /// </summary>
        /// <param name="curve">Source curve</param>
        /// <param name="scale">Scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public static NurbsCurve Scale(NurbsCurve curve, double scale, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(scale, scale, scale, center.Value)
                : Matrix4x4.CreateScale(scale, scale, scale);
            return Transform(curve, matrix);
        }

        /// <summary>
        /// (en) Scale curve non-uniformly (non-destructive)
        /// (ja) 曲線を非一様にスケール（非破壊的）
        /// </summary>
        /// <param name="curve">Source curve</param>
        /// <param name="sx">X scale factor</param>
        /// <param name="sy">Y scale factor</param>
        /// <param name="sz">Z scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public static NurbsCurve Scale(NurbsCurve curve, double sx, double sy, double sz, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(sx, sy, sz, center.Value)
                : Matrix4x4.CreateScale(sx, sy, sz);
            return Transform(curve, matrix);
        }

        /// <summary>
        /// (en) Scale surface uniformly (non-destructive)
        /// (ja) サーフェスを一様にスケール（非破壊的）
        /// </summary>
        /// <param name="surface">Source surface</param>
        /// <param name="scale">Scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public static NurbsSurface Scale(NurbsSurface surface, double scale, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(scale, scale, scale, center.Value)
                : Matrix4x4.CreateScale(scale, scale, scale);
            return Transform(surface, matrix);
        }

        /// <summary>
        /// (en) Scale surface non-uniformly (non-destructive)
        /// (ja) サーフェスを非一様にスケール（非破壊的）
        /// </summary>
        /// <param name="surface">Source surface</param>
        /// <param name="sx">X scale factor</param>
        /// <param name="sy">Y scale factor</param>
        /// <param name="sz">Z scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public static NurbsSurface Scale(NurbsSurface surface, double sx, double sy, double sz, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(sx, sy, sz, center.Value)
                : Matrix4x4.CreateScale(sx, sy, sz);
            return Transform(surface, matrix);
        }

        /// <summary>
        /// (en) Scale volume uniformly (non-destructive)
        /// (ja) ボリュームを一様にスケール（非破壊的）
        /// </summary>
        /// <param name="volume">Source volume</param>
        /// <param name="scale">Scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public static NurbsVolume Scale(NurbsVolume volume, double scale, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(scale, scale, scale, center.Value)
                : Matrix4x4.CreateScale(scale, scale, scale);
            return Transform(volume, matrix);
        }

        /// <summary>
        /// (en) Scale volume non-uniformly (non-destructive)
        /// (ja) ボリュームを非一様にスケール（非破壊的）
        /// </summary>
        /// <param name="volume">Source volume</param>
        /// <param name="sx">X scale factor</param>
        /// <param name="sy">Y scale factor</param>
        /// <param name="sz">Z scale factor</param>
        /// <param name="center">Center of scaling (default: origin)</param>
        public static NurbsVolume Scale(NurbsVolume volume, double sx, double sy, double sz, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(sx, sy, sz, center.Value)
                : Matrix4x4.CreateScale(sx, sy, sz);
            return Transform(volume, matrix);
        }

        #endregion

        #region Destructive transformations (modify in-place)

        /// <summary>
        /// (en) Transform curve using transformation matrix (destructive)
        /// (ja) 変換行列を使用して曲線を変換（破壊的）
        /// </summary>
        /// <param name="curve">Curve to transform</param>
        /// <param name="matrix">Transformation matrix</param>
        public static void TransformInPlace(NurbsCurve curve, Matrix4x4 matrix)
        {
            Guard.ThrowIfNull(curve, nameof(curve));

            for (int i = 0; i < curve.ControlPoints.Length; i++)
            {
                Vector3Double transformedPos = matrix.TransformPoint(curve.ControlPoints[i].Position);
                curve.ControlPoints[i].Position = transformedPos;
            }
        }

        /// <summary>
        /// (en) Transform surface using transformation matrix (destructive)
        /// (ja) 変換行列を使用してサーフェスを変換（破壊的）
        /// </summary>
        /// <param name="surface">Surface to transform</param>
        /// <param name="matrix">Transformation matrix</param>
        public static void TransformInPlace(NurbsSurface surface, Matrix4x4 matrix)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            for (int i = 0; i < surface.ControlPoints.Length; i++)
            {
                for (int j = 0; j < surface.ControlPoints[i].Length; j++)
                {
                    Vector3Double transformedPos = matrix.TransformPoint(surface.ControlPoints[i][j].Position);
                    surface.ControlPoints[i][j].Position = transformedPos;
                }
            }
        }

        /// <summary>
        /// (en) Transform volume using transformation matrix (destructive)
        /// (ja) 変換行列を使用してボリュームを変換（破壊的）
        /// </summary>
        /// <param name="volume">Volume to transform</param>
        /// <param name="matrix">Transformation matrix</param>
        public static void TransformInPlace(NurbsVolume volume, Matrix4x4 matrix)
        {
            Guard.ThrowIfNull(volume, nameof(volume));

            for (int i = 0; i < volume.ControlPoints.Length; i++)
            {
                for (int j = 0; j < volume.ControlPoints[i].Length; j++)
                {
                    for (int k = 0; k < volume.ControlPoints[i][j].Length; k++)
                    {
                        Vector3Double transformedPos = matrix.TransformPoint(volume.ControlPoints[i][j][k].Position);
                        volume.ControlPoints[i][j][k].Position = transformedPos;
                    }
                }
            }
        }

        /// <summary>
        /// (en) Translate curve (destructive)
        /// (ja) 曲線を平行移動（破壊的）
        /// </summary>
        public static void TranslateInPlace(NurbsCurve curve, Vector3Double delta)
        {
            TransformInPlace(curve, Matrix4x4.CreateTranslation(delta));
        }

        /// <summary>
        /// (en) Translate curve (destructive)
        /// (ja) 曲線を平行移動（破壊的）
        /// </summary>
        public static void TranslateInPlace(NurbsCurve curve, double dx, double dy, double dz)
        {
            TransformInPlace(curve, Matrix4x4.CreateTranslation(dx, dy, dz));
        }

        /// <summary>
        /// (en) Translate surface (destructive)
        /// (ja) サーフェスを平行移動（破壊的）
        /// </summary>
        public static void TranslateInPlace(NurbsSurface surface, Vector3Double delta)
        {
            TransformInPlace(surface, Matrix4x4.CreateTranslation(delta));
        }

        /// <summary>
        /// (en) Translate surface (destructive)
        /// (ja) サーフェスを平行移動（破壊的）
        /// </summary>
        public static void TranslateInPlace(NurbsSurface surface, double dx, double dy, double dz)
        {
            TransformInPlace(surface, Matrix4x4.CreateTranslation(dx, dy, dz));
        }

        /// <summary>
        /// (en) Translate volume (destructive)
        /// (ja) ボリュームを平行移動（破壊的）
        /// </summary>
        public static void TranslateInPlace(NurbsVolume volume, Vector3Double delta)
        {
            TransformInPlace(volume, Matrix4x4.CreateTranslation(delta));
        }

        /// <summary>
        /// (en) Translate volume (destructive)
        /// (ja) ボリュームを平行移動（破壊的）
        /// </summary>
        public static void TranslateInPlace(NurbsVolume volume, double dx, double dy, double dz)
        {
            TransformInPlace(volume, Matrix4x4.CreateTranslation(dx, dy, dz));
        }

        /// <summary>
        /// (en) Rotate curve around axis (destructive)
        /// (ja) 曲線を軸周りに回転（破壊的）
        /// </summary>
        public static void RotateInPlace(NurbsCurve curve, Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateRotation(axis, angle, center.Value)
                : Matrix4x4.CreateRotation(axis, angle);
            TransformInPlace(curve, matrix);
        }

        /// <summary>
        /// (en) Rotate surface around axis (destructive)
        /// (ja) サーフェスを軸周りに回転（破壊的）
        /// </summary>
        public static void RotateInPlace(NurbsSurface surface, Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateRotation(axis, angle, center.Value)
                : Matrix4x4.CreateRotation(axis, angle);
            TransformInPlace(surface, matrix);
        }

        /// <summary>
        /// (en) Rotate volume around axis (destructive)
        /// (ja) ボリュームを軸周りに回転（破壊的）
        /// </summary>
        public static void RotateInPlace(NurbsVolume volume, Vector3Double axis, double angle, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateRotation(axis, angle, center.Value)
                : Matrix4x4.CreateRotation(axis, angle);
            TransformInPlace(volume, matrix);
        }

        /// <summary>
        /// (en) Scale curve uniformly (destructive)
        /// (ja) 曲線を一様にスケール（破壊的）
        /// </summary>
        public static void ScaleInPlace(NurbsCurve curve, double scale, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(scale, scale, scale, center.Value)
                : Matrix4x4.CreateScale(scale, scale, scale);
            TransformInPlace(curve, matrix);
        }

        /// <summary>
        /// (en) Scale curve non-uniformly (destructive)
        /// (ja) 曲線を非一様にスケール（破壊的）
        /// </summary>
        public static void ScaleInPlace(NurbsCurve curve, double sx, double sy, double sz, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(sx, sy, sz, center.Value)
                : Matrix4x4.CreateScale(sx, sy, sz);
            TransformInPlace(curve, matrix);
        }

        /// <summary>
        /// (en) Scale surface uniformly (destructive)
        /// (ja) サーフェスを一様にスケール（破壊的）
        /// </summary>
        public static void ScaleInPlace(NurbsSurface surface, double scale, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(scale, scale, scale, center.Value)
                : Matrix4x4.CreateScale(scale, scale, scale);
            TransformInPlace(surface, matrix);
        }

        /// <summary>
        /// (en) Scale surface non-uniformly (destructive)
        /// (ja) サーフェスを非一様にスケール（破壊的）
        /// </summary>
        public static void ScaleInPlace(NurbsSurface surface, double sx, double sy, double sz, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(sx, sy, sz, center.Value)
                : Matrix4x4.CreateScale(sx, sy, sz);
            TransformInPlace(surface, matrix);
        }

        /// <summary>
        /// (en) Scale volume uniformly (destructive)
        /// (ja) ボリュームを一様にスケール（破壊的）
        /// </summary>
        public static void ScaleInPlace(NurbsVolume volume, double scale, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(scale, scale, scale, center.Value)
                : Matrix4x4.CreateScale(scale, scale, scale);
            TransformInPlace(volume, matrix);
        }

        /// <summary>
        /// (en) Scale volume non-uniformly (destructive)
        /// (ja) ボリュームを非一様にスケール（破壊的）
        /// </summary>
        public static void ScaleInPlace(NurbsVolume volume, double sx, double sy, double sz, Vector3Double? center = null)
        {
            Matrix4x4 matrix = center.HasValue
                ? Matrix4x4.CreateScale(sx, sy, sz, center.Value)
                : Matrix4x4.CreateScale(sx, sy, sz);
            TransformInPlace(volume, matrix);
        }

        #endregion
    }
}
