using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Evaluation
{
    /// <summary>
    /// (en) Evaluator for NURBS surfaces
    /// (ja) NURBSサーフェスの評価クラス
    /// </summary>
    public class SurfaceEvaluator:BasicEvaluator
    {
        /// <summary>
        /// (en) Evaluates the position on the NURBS surface at the specified parameters u and v. The range is the same as the knot vector's minimum and maximum values.
        /// (ja) 指定したパラメータ u と v でNURBSサーフェス上の位置を評価します。レンジはノットベクトルの最小値と最大値と同じです。
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static (double x, double y, double z) Evaluate(NurbsSurface surface, double u, double v)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;

            var controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;

            var knotsU = surface.KnotVectorU.Knots;
            var knotsV = surface.KnotVectorV.Knots;

            // find u v knot span index
            int spanU = FindSpan(degreeU, knotsU, u);
            int spanV = FindSpan(degreeV, knotsV, v);

            // de Boor's algorithm in U direction
            Vector4Double[] temp = new Vector4Double[nV];
            for (int j = 0; j < nV; j++)
            {
                Vector4Double[] row = new Vector4Double[nU];
                for (int i = 0; i < nU; i++)
                {
                    row[i] = controlPoints[i][j].HomogeneousPosition;
                }
                temp[j] = DeBoor(degreeU, knotsU,spanU, row, u);
            }

            // de Boor's algorithm in V direction
            Vector4Double Sv = DeBoor(degreeV, knotsV,spanV, temp, v);

            return (Sv.X/Sv.W, Sv.Y / Sv.W, Sv.Z / Sv.W);
        }

        //TODO: Optimize using different numerical integration methods
        // not good for high curvature surface
        public static double SurfaceArea(NurbsSurface surface, double start_u, double end_u, double start_v, double end_v, double epsilon = 0.01)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));
            double area = 0.0;
            for (double u = start_u; u < end_u; u += epsilon)
            {

                for (double v = start_v; v < end_v; v += epsilon)
                {
                    Vector3Double p1, p2, p3,p4;
                    //pick small triangle area
                    var res = Evaluate(surface, u, v);
                    p1 = new Vector3Double(res.x, res.y, res.z);

                    if (u + epsilon > end_u)
                        res = Evaluate(surface, end_u, v);
                    else
                        res = Evaluate(surface, u + epsilon, v);
                    p2 = new Vector3Double(res.x, res.y, res.z);
                    
                    if (v + epsilon > end_v)
                        res = Evaluate(surface, u, end_v);
                    else
                        res = Evaluate(surface, u, v + epsilon);
                    p3 = new Vector3Double(res.x, res.y, res.z);

                    if (u + epsilon > end_u)
                    {
                        if (v + epsilon > end_v)
                            res = Evaluate(surface, end_u, end_v);
                        else
                            res = Evaluate(surface, end_u, v + epsilon);
                    }
                    else if(v + epsilon > end_v)
                        res = Evaluate(surface, u + epsilon, end_v);
                    else
                        res = Evaluate(surface, u + epsilon, v + epsilon);

                    p4 = new Vector3Double(res.x, res.y, res.z);
                    // S = ||AB x AC||
                    Vector3Double vec1 = p2 - p1;
                    Vector3Double vec2 = p3 - p1;
                    Vector3Double crossProduct = Vector3Double.Cross(vec1, vec2);
                    area += crossProduct.magnitude / 2; // Triangle area

                    vec1 = p2 - p4;
                    vec2 = p3 - p4;
                    crossProduct = Vector3Double.Cross(vec1, vec2);

                    area += crossProduct.magnitude / 2; // Triangle area

                }
            }
            return area;
        }
    }

}
