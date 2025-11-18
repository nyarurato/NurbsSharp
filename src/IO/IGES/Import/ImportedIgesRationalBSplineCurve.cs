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
    internal class ImportedIgesRationalBSplineCurve: IIgesImportEntity
    {
        public int EntityType => 126;
        public NurbsCurve? Curve { get; private set; }
        public IGeometry Geometry => Curve!;

        public ImportedIgesRationalBSplineCurve()
        {
            Curve = null!;
        }

        public bool ParseParameterEntity(string[] entity_str)
        {
            try
            {
                // Each parameter row is 72 chars: first 64 chars are parameter data, last 8 are pointer
                var paramData = new StringBuilder();
                foreach (var row in entity_str)
                {
                    if (row.Length >= 64)
                        paramData.Append(row.Substring(0, 64));
                    else
                        paramData.Append(row);
                }

                // tokens separated by comma, terminated by semicolon
                var joined = paramData.ToString();
                joined = joined.Replace(";", ",");
                var rawTokens = joined.Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries)
                                       .Select(s => s.Trim())
                                       .ToArray();

                if (rawTokens.Length < 7)
                    return false;

                // tokens: [EntityType, K, M, PROP1, PROP2, PROP3, PROP4, knots..., weights..., control points..., V0, V1]
                int idx = 0;
                // skip entity type
                idx++; // entity type
                int K = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                int M = int.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                // read and ignore props
                idx += 4;

                int knotCount = K + M + 2;
                if (idx + knotCount > rawTokens.Length)
                    return false;

                var knots = new double[knotCount];
                for (int i = 0; i < knotCount; i++)
                {
                    knots[i] = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                }

                int cpCount = K + 1;
                if (idx + cpCount > rawTokens.Length)
                    return false;

                var weights = new double[cpCount];
                for (int i = 0; i < cpCount; i++)
                {
                    weights[i] = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                }

                var controlPoints = new ControlPoint[cpCount];
                for (int i = 0; i < cpCount; i++)
                {
                    double x = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                    double y = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                    double z = double.Parse(rawTokens[idx++], CultureInfo.InvariantCulture);
                    controlPoints[i] = new ControlPoint(x, y, z, weights[i]);
                }

                // V0,V1 follow (we can ignore)
                // Build objects
                var kv = new KnotVector(knots, M);
                Curve = new NurbsCurve(M, kv, controlPoints);
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
