using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Tesselation
{
    /// <summary>
    /// (en)Tessellator for NURBS surfaces
    /// (ja)NURBSサーフェスのテッセレーター
    /// </summary>
    public class SurfaceTessellator
    {
        /// <summary>
        /// (en)Tessellate NURBS surface into triangle mesh
        /// (ja)NURBSサーフェスを三角形メッシュにする
        /// </summary>
        /// <param name="surface"></param>
        /// <param name="numPointsU"></param>
        /// <param name="numPointsV"></param>
        /// <returns></returns>
        public static Mesh Tessellate(NurbsSurface surface,int numPointsU,int numPointsV)
        {
            Vector3Double[] vertices = new Vector3Double[numPointsU * numPointsV];
            int[] indexes = new int[(numPointsU - 1) * (numPointsV - 1) * 6];
            double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
            double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
            double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

            //TODO: Parallelize these loop
            for (int i = 0; i < numPointsU; i++)
            {
                double u = uStart + (uEnd - uStart) * i / (numPointsU - 1);
                for (int j = 0; j < numPointsV; j++)
                {
                    double v = vStart + (vEnd - vStart) * j / (numPointsV - 1);
                    var point = surface.GetPos(u, v);
                    vertices[i * numPointsV + j] = point;
                }
            }

            int index = 0;
            for (int i = 0; i < numPointsU - 1; i++)
            {
                for (int j = 0; j < numPointsV - 1; j++)
                {
                    int v0 = i * numPointsV + j;
                    int v1 = (i + 1) * numPointsV + j;
                    int v2 = (i + 1) * numPointsV + (j + 1);
                    int v3 = i * numPointsV + (j + 1);
                    // First triangle
                    indexes[index++] = v0;
                    indexes[index++] = v1;
                    indexes[index++] = v2;
                    // Second triangle
                    indexes[index++] = v0;
                    indexes[index++] = v2;
                    indexes[index++] = v3;
                }
            }
            return new Mesh(vertices, indexes);

        }
    }
}
