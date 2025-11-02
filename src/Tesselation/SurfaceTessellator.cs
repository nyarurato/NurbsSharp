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
        public static Mesh Tessellate(NurbsSurface surface,int PointsU,int PointsV)
        {
            Vector3Double[] vertices = new Vector3Double[PointsU * PointsV];
            int[] indexes = new int[(PointsU - 1) * (PointsV - 1) * 6];
            double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
            double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
            double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];
            
            for (int i = 0; i < PointsU; i++)
            {
                double u = uStart + (uEnd - uStart) * i / (PointsU - 1);
                for (int j = 0; j < PointsV; j++)
                {
                    double v = vStart + (vEnd - vStart) * j / (PointsV - 1);
                    var point = surface.GetPos(u, v);
                    vertices[i * PointsV + j] = point;
                }
            }

            int index = 0;
            for (int i = 0; i < PointsU - 1; i++)
            {
                for (int j = 0; j < PointsV - 1; j++)
                {
                    int v0 = i * PointsV + j;
                    int v1 = (i + 1) * PointsV + j;
                    int v2 = (i + 1) * PointsV + (j + 1);
                    int v3 = i * PointsV + (j + 1);
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
