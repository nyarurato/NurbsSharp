using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;
using System.Data.Common;


namespace NurbsSharp.Geometry.Primitive
{
    /// <summary>
    /// (en)Factory class for creating primitive NURBS geometries
    /// (ja)基本的なNURBSジオメトリを作成するためのファクトリークラス
    /// </summary>
    public static class PrimitiveFactory
    {
        /// <summary>
        /// (en) Create a NURBS circle on the XY plane
        /// (ja) XY平面にNURBSの円を作成する
        /// </summary>
        /// <param name="radius"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static NurbsCurve CreateCircle(double radius)
        {
            Guard.ThrowIfNegativeOrZero(radius, nameof(radius));
            // A NURBS circle can be represented as a rational quadratic B-spline with 9 control points
            int degree = 2;
            ControlPoint[] controlPoints = new ControlPoint[9];
            double weight = Math.Sqrt(2) / 2.0;
            controlPoints[0] = new ControlPoint(radius, 0.0, 0.0, 1.0);
            controlPoints[1] = new ControlPoint(radius, radius, 0.0, weight);
            controlPoints[2] = new ControlPoint(0.0, radius, 0.0, 1.0);
            controlPoints[3] = new ControlPoint(-radius, radius, 0.0, weight);
            controlPoints[4] = new ControlPoint(-radius, 0.0, 0.0, 1.0);
            controlPoints[5] = new ControlPoint(-radius, -radius, 0.0, weight);
            controlPoints[6] = new ControlPoint(0.0, -radius, 0.0, 1.0);
            controlPoints[7] = new ControlPoint(radius, -radius, 0.0, weight);
            controlPoints[8] = new ControlPoint(radius, 0.0, 0.0, 1.0);
            double[] knots = new double[]
            {
                0.0, 0.0, 0.0,
                1.0/4.0, 1.0/4.0,
                1.0/2.0, 1.0/2.0,
                3.0/4.0, 3.0/4.0,
                1.0, 1.0, 1.0
            };
            var knotVector = new KnotVector(knots,degree);
            var circleCurve = new NurbsCurve(degree, knotVector, controlPoints);
            return circleCurve;
        }

        /// <summary>
        /// Create a straight line segment as a degree-1 NURBS curve
        /// </summary>
        public static NurbsCurve CreateLine(Vector3Double p0, Vector3Double p1)
        {
            var cps = new ControlPoint[2];
            cps[0] = new ControlPoint(p0, 1.0);
            cps[1] = new ControlPoint(p1, 1.0);
            int degree = 1;
            var kv = KnotVector.GetClampedKnot(degree, cps.Length);
            return new NurbsCurve(degree, kv, cps);
        }


        /// <summary>
        /// (en) Create a NURBS surface representing a cylinder aligned along the Z axis
        /// (ja) Z軸に沿った円柱を表すNURBSサーフェスを作成する
        /// </summary>
        /// <param name="radius"></param>
        /// <param name="height"></param>
        /// <param name="isGenerateTopBottom"></param>
        /// <returns></returns>
        /// <exception cref="NotImplementedException"></exception>
        public static List<NurbsSurface> CreateCylinder(double radius, double height,bool isGenerateTopBottom=false)
        {
            Guard.ThrowIfNegativeOrZero(radius, nameof(radius));
            var curve = CreateCircle(radius);

            int degreeU = 1; // height direction (two rows)
            int degreeV = curve.Degree; // around
            int nU = 2;
            int nV = curve.ControlPoints.Length; // duplicate first column at end for seam

            var kvU = KnotVector.GetClampedKnot(degreeU, nU);
            var kvV = curve.KnotVector; // use circle's knot vector

            ControlPoint[][] cps = new ControlPoint[nU][];
            for (int iu = 0; iu < nU; iu++)
            {
                cps[iu] = new ControlPoint[nV];
                double z = (iu == 0) ? -height / 2.0 : height / 2.0;
                for (int iv = 0; iv < nV; iv++)
                {
                    var cpCircle = curve.ControlPoints[iv];
                    cps[iu][iv] = new ControlPoint(cpCircle.Position.X, cpCircle.Position.Y, z, cpCircle.Weight);
                }
            }
            var surfaces = new List<NurbsSurface>();
            surfaces.Add( new NurbsSurface(degreeU, degreeV, kvU, kvV, cps));

            if(isGenerateTopBottom){
                var topSurface = CreateCircleSurface(radius);
                topSurface.Translate(0.0, 0.0, height / 2.0);
                surfaces.Add(topSurface);
                var bottomSurface = CreateCircleSurface(radius);
                bottomSurface.Translate(0.0, 0.0, -height / 2.0);
                surfaces.Add(bottomSurface);
            }

            return surfaces;
        }

        /// <summary>
        /// (en) Create a NURBS surface representing a circle surface (a disk) on the XY plane
        /// (ja) XY平面に円形サーフェス（円盤）を表すNURBSサーフェスを作成する
        /// </summary>
        /// <param name="radius"></param>
        /// <returns></returns>
        public static NurbsSurface CreateCircleSurface(double radius)
        {
            //TDOO: It is not certain whether this implementation is correct.

            Guard.ThrowIfNegativeOrZero(radius, nameof(radius));

            // outline circle on XY plane
            var circle = CreateCircle(radius);

            int degreeU = 1; // center to outline direction
            int degreeV = circle.Degree; // around direction
            int nU = 2; // center to outline
            int nV = circle.ControlPoints.Length;

            var kvU = KnotVector.GetClampedKnot(degreeU, nU);
            var kvV = circle.KnotVector;

            ControlPoint[][] cps = new ControlPoint[nU][];
            for (int iu = 0; iu < nU; iu++)
            {
                cps[iu] = new ControlPoint[nV];
                for (int iv = 0; iv < nV; iv++)
                {
                    if (iu == 0)
                    {
                        // center point
                        cps[iu][iv] = new ControlPoint(0.0, 0.0, 0.0, 1.0);
                    }
                    else
                    {
                        // outline circle points
                        var cp = circle.ControlPoints[iv];
                        cps[iu][iv] = new ControlPoint(cp.Position.X, cp.Position.Y, cp.Position.Z, cp.Weight);
                    }
                }
            }

            return new NurbsSurface(degreeU, degreeV, kvU, kvV, cps);
        }

        /// <summary>
        /// (en) Create a NURBS surface representing a sphere centered at the origin
        /// (ja) 原点を中心とした球体を表すNURBSサーフェスを作成する
        /// </summary>
        /// <param name="radius"></param>
        /// <returns></returns>
        public static NurbsSurface CreateSphere(double radius)
        {
            Guard.ThrowIfNegativeOrZero(radius, nameof(radius));
            int degreeU = 3;
            int degreeV = 3;
            int cpU = 4;
            int cpV = 11;

            return null; // TODO: implement this
        }

        /// <summary>
        /// (en) Create a degree-1 2x2 NURBS surface (a single face) defined by four corner points
        /// (ja) 4つのコーナーポイントで定義される1次2x2のNURBSサーフェス（単一の面）を作成する
        /// </summary>
        /// <param name="p00"></param>
        /// <param name="p01"></param>
        /// <param name="p10"></param>
        /// <param name="p11"></param>
        /// <returns></returns>
        public static NurbsSurface CreateFace(Vector3Double p00, Vector3Double p01, Vector3Double p10, Vector3Double p11)
        {
            var kv = KnotVector.GetClampedKnot(1, 2); // degree1, 2 control points per direction

            var cps = new ControlPoint[2][];
            cps[0] = new ControlPoint[2]; cps[1] = new ControlPoint[2];
            cps[0][0] = new ControlPoint(p00, 1.0);
            cps[0][1] = new ControlPoint(p01, 1.0);
            cps[1][0] = new ControlPoint(p10, 1.0);
            cps[1][1] = new ControlPoint(p11, 1.0);
            return new NurbsSurface(1, 1, kv, kv, cps);
        }

        /// <summary>
        /// Create a box as six separate NURBS surfaces (faces). Each face is a degree-1 2x2 surface.
        /// Returns an array of 6 surfaces in the order: +X, -X, +Y, -Y, +Z, -Z
        /// </summary>
        public static NurbsSurface[] CreateBox(double width, double depth, double height, Vector3Double? center = null)
        {
            Guard.ThrowIfNegativeOrZero(width, nameof(width));
            Guard.ThrowIfNegativeOrZero(depth, nameof(depth));
            Guard.ThrowIfNegativeOrZero(height, nameof(height));
            var c = center ?? Vector3Double.Zero;

            double hx = width / 2.0;
            double hy = depth / 2.0;
            double hz = height / 2.0;

            var kv = KnotVector.GetClampedKnot(1, 2); // degree1, 2 control points per direction
            var faces = new NurbsSurface[6];

            // +X face (x = +hx)
            faces[0] = CreateFace(
                new Vector3Double(c.X + hx, c.Y - hy, c.Z - hz),
                new Vector3Double(c.X + hx, c.Y + hy, c.Z - hz),
                new Vector3Double(c.X + hx, c.Y - hy, c.Z + hz),
                new Vector3Double(c.X + hx, c.Y + hy, c.Z + hz)
            );
            // -X face
            faces[1] = CreateFace(
                new Vector3Double(c.X - hx, c.Y + hy, c.Z - hz),
                new Vector3Double(c.X - hx, c.Y - hy, c.Z - hz),
                new Vector3Double(c.X - hx, c.Y + hy, c.Z + hz),
                new Vector3Double(c.X - hx, c.Y - hy, c.Z + hz)
            );
            // +Y face
            faces[2] = CreateFace(
                new Vector3Double(c.X - hx, c.Y + hy, c.Z - hz),
                new Vector3Double(c.X + hx, c.Y + hy, c.Z - hz),
                new Vector3Double(c.X - hx, c.Y + hy, c.Z + hz),
                new Vector3Double(c.X + hx, c.Y + hy, c.Z + hz)
            );
            // -Y face
            faces[3] = CreateFace(
                new Vector3Double(c.X + hx, c.Y - hy, c.Z - hz),
                new Vector3Double(c.X - hx, c.Y - hy, c.Z - hz),
                new Vector3Double(c.X + hx, c.Y - hy, c.Z + hz),
                new Vector3Double(c.X - hx, c.Y - hy, c.Z + hz)
            );
            // +Z face
            faces[4] = CreateFace(
                new Vector3Double(c.X - hx, c.Y - hy, c.Z + hz),
                new Vector3Double(c.X + hx, c.Y - hy, c.Z + hz),
                new Vector3Double(c.X - hx, c.Y + hy, c.Z + hz),
                new Vector3Double(c.X + hx, c.Y + hy, c.Z + hz)
            );
            // -Z face
            faces[5] = CreateFace(
                new Vector3Double(c.X - hx, c.Y + hy, c.Z - hz),
                new Vector3Double(c.X + hx, c.Y + hy, c.Z - hz),
                new Vector3Double(c.X - hx, c.Y - hy, c.Z - hz),
                new Vector3Double(c.X + hx, c.Y - hy, c.Z - hz)
            );

            return faces;
        }
    }
}
