using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Globalization;
using NurbsSharp.IO;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.IO.IGES
{
    internal class ImportedIgesRationalBSplineSurface: IIgesImportEntity
    {
        public int EntityType => 128;
        public NurbsSurface? Surface { get; private set; }
        public IGeometry Geometry => Surface!;

        public ImportedIgesRationalBSplineSurface()
        {
            Surface = null!;
        }

        public bool ParseParameterEntity(string[] entity_str)
        {
            try
            {
                var paramBuilder = new StringBuilder();
                foreach (var row in entity_str)
                {
                    if (row.Length >= 64)
                        paramBuilder.Append(row.AsSpan(0, 64));
                    else
                        paramBuilder.Append(row);
                }

                var joined = paramBuilder.ToString();
                joined = joined.Replace(";", ",");
                var rawTokens = joined.Split([ ',' ], StringSplitOptions.RemoveEmptyEntries)
                                       .Select(s => s.Trim())
                                       .ToArray();

                if (rawTokens.Length < 10)
                    return false;

                int idx = 0;
                // skip entity type
                idx++;
                int k1 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int k2 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int m1 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int m2 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                // p1..p5 (ignore now)
                int prop1 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int prop2 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int prop3 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int prop4 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int prop5 = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);

                int knotUCount = k1 + m1 + 2;
                int knotVCount = k2 + m2 + 2;

                if (idx + knotUCount + knotVCount > rawTokens.Length)
                    return false;

                var knotsU = new double[knotUCount];
                for (int i = 0; i < knotUCount; i++)
                    knotsU[i] = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);

                var knotsV = new double[knotVCount];
                for (int i = 0; i < knotVCount; i++)
                    knotsV[i] = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);

                int cpU = k1 + 1;
                int cpV = k2 + 1;
                int cpCount = cpU * cpV;

                if (idx + cpCount > rawTokens.Length)
                    return false;

                var weights = new double[cpCount];
                for (int i = 0; i < cpCount; i++)
                    weights[i] = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);

                if (idx + 3 * cpCount > rawTokens.Length)
                    return false;

                // Control points are flattened U major then V (export loop v outer, u inner)
                // Export loop: for v=0..k2, for u=0..k1 write coords
                var controlPoints = new ControlPoint[cpU][];
                for (int u = 0; u < cpU; u++)
                    controlPoints[u] = new ControlPoint[cpV];

                for (int v = 0; v < cpV; v++)
                {
                    for (int u = 0; u < cpU; u++)
                    {
                        double x = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                        double y = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                        double z = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                        int flatIndex = v * cpU + u;
                        controlPoints[u][v] = new ControlPoint(x, y, z, weights[flatIndex]);
                    }
                }

                // Create knot vectors and surface
                var kvU = new KnotVector(knotsU, m1);
                var kvV = new KnotVector(knotsV, m2);
                Surface = new NurbsSurface(m1, m2, kvU, kvV, controlPoints);
                return true;
            }
            catch
            {
                return false;
            }
        }
        /// <summary>
        /// (en) Parses the directory entry data from the given string.
        /// (ja) 指定された文字列からディレクトリエントリデータを解析します。
        /// </summary>
        /// <param name="directory_entry_str"></param>
        /// <returns></returns>
        public (bool result, int parameter_start_pointer, int parameter_row_count) ParseDirectoryEntry(string[] directory_entry_str)
        {
            try
            {
                if (directory_entry_str == null || directory_entry_str.Length < 2)
                    return (false, -1, -1);

                var row1 = directory_entry_str[0];
                var row2 = directory_entry_str[1];

                if (row1.Length < 16 || row2.Length < 32)
                    return (false, -1, -1);

                var ptrStr = row1.Substring(8, 8).Trim();
                var countStr = row2.Substring(24, 8).Trim();

                if (!int.TryParse(ptrStr, out int paramPointer))
                    return (false, -1, -1);
                if (!int.TryParse(countStr, out int paramLineCount))
                    return (false, -1, -1);

                return (true, paramPointer, paramLineCount);
            }
            catch
            {
                return (false, -1, -1);
            }
        }

    }
}
