using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Evaluation
{
    /// <summary>
    /// (en) Evaluator for NURBS volumes
    /// (ja) NURBSボリュームの評価クラス
    /// </summary>
    public class VolumeEvaluator : BasicEvaluator
    {
        /// <summary>
        /// (en) Evaluates the position on the NURBS volume at the specified parameters u, v and w. The range is the same as the knot vector's minimum and maximum values.
        /// (ja) 指定したパラメータ u、v、w でNURBSボリューム上の位置を評価します。レンジはノットベクトルの最小値と最大値と同じです。
        /// </summary>
        /// <param name="volume"></param>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <param name="w"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static (double x, double y, double z) Evaluate(NurbsVolume volume, double u, double v, double w)
        {
            if (volume == null)
                throw new ArgumentNullException(nameof(volume));
            int degreeU = volume.DegreeU;
            int degreeV = volume.DegreeV;
            int degreeW = volume.DegreeW;
            var controlPoints = volume.ControlPoints;
            int nU = controlPoints.Length;
            int nV = controlPoints[0].Length;
            int nW = controlPoints[0][0].Length;
            var knotsU = volume.KnotVectorU.Knots;
            var knotsV = volume.KnotVectorV.Knots;
            var knotsW = volume.KnotVectorW.Knots;
            // find u v w knot span index
            int spanU = FindSpan(degreeU, knotsU, u);
            int spanV = FindSpan(degreeV, knotsV, v);
            int spanW = FindSpan(degreeW, knotsW, w);
            // de Boor's algorithm in U direction
            Vector4Double[][] tempUV = new Vector4Double[nV][];
            for(int j = 0; j < nV; j++)
            {
                tempUV[j] = new Vector4Double[nW];
            }

            for (int j = 0; j < nV; j++)
            {
                for (int k = 0; k < nW; k++)
                {
                    Vector4Double[] row = new Vector4Double[nU];
                    for (int i = 0; i < nU; i++)
                    {
                        row[i] = controlPoints[i][j][k].HomogeneousPosition;
                    }
                    tempUV[j][k] = DeBoor(degreeU, knotsU, spanU, row, u);
                }
            }
            // de Boor's algorithm in V direction
            Vector4Double[] tempV = new Vector4Double[nW];
            for (int k = 0; k < nW; k++)
            {
                Vector4Double[] column = new Vector4Double[nV];
                for (int j = 0; j < nV; j++)
                {
                    column[j] = tempUV[j][k];
                }
                tempV[k] = DeBoor(degreeV, knotsV, spanV, column, v);
            }
            // de Boor's algorithm in W direction
            Vector4Double Sw = DeBoor(degreeW, knotsW, spanW, tempV, w);
            return (Sw.X / Sw.W, Sw.Y / Sw.W, Sw.Z / Sw.W);
        }

    }
}
