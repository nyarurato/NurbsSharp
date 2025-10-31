using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace NurbsSharp.Evaluation
{
    public class SurfaceEvaluator
    {
        public static (double x, double y, double z) Evaluate(NurbsSurface surface, double u, double v)
        {
            if (surface == null)
                throw new ArgumentNullException(nameof(surface));

            int degreeU = surface.DegreeU;
            int degreeV = surface.DegreeV;

            var controlPoints = surface.ControlPoints;

            int nU = controlPoints.Length - 1;
            int nV = controlPoints[0].Length - 1;

            var knotsU = surface.KnotVectorU.Knots;
            var knotsV = surface.KnotVectorV.Knots;

            // find u v knot span index
            int spanU = FindSpan(degreeU, knotsU, u);
            int spanV = FindSpan(degreeV, knotsV, v);

            // de Boor's algorithm in U direction
            

            return (0,0,0);
        }

        // Find knot span index
        static int FindSpan(int degree, double[] knots, double t)
        {
            int k = -1;
            int s = 0;
            for (int i = degree; i < (knots.Length - 1); i++)
            {
                if (t >= knots[i] && t < knots[i + 1])
                {
                    k = i;
                    break;
                }
                else if (t == knots[i + 1])
                {
                    s++;
                }
            }
            if (t == knots[knots.Length - 1])
            {
                k = knots.Length - s - 1;
            }

            if (k == -1)
            {
                throw new ArgumentOutOfRangeException(nameof(t), $"Parameter 't' is out of the knot vector range. val:{t}");
            }

            return k;
        }
        
    }

}
