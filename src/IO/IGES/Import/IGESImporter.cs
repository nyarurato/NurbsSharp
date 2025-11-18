using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;
using System.IO;

namespace NurbsSharp.IO.IGES
{
    /// <summary>
    /// (en) IGES Importer
    /// (ja) IGESインポーター
    /// </summary>
    public class IGESImporter
    {
        /// <summary>
        /// (en) Imports IGES file asynchronously
        /// (ja) IGESファイルを非同期にインポートします
        /// </summary>
        /// <param name="filePath"></param>
        /// <returns></returns>
        public static async Task<List<IGeometry>> ImportAsync(string filePath)
        {
            using (var stream = new StreamReader(filePath, Encoding.ASCII))
            {
                return await ImportAsync(stream);
            }
        }

        /// <summary>
        /// (en) Imports IGES from a StreamReader asynchronously
        /// (ja) StreamReaderからIGESを非同期にインポートします
        /// </summary>
        /// <param name="stream"></param>
        /// <returns></returns>
        /// <exception cref="FormatException"></exception>
        public static async Task<List<IGeometry>> ImportAsync(StreamReader stream)
        {
            List<IGESROW> Directory = new List<IGESROW>();
            List<IGESROW> Parameter = new List<IGESROW>();


            while (!stream.EndOfStream)
            {
                string? line = await stream.ReadLineAsync();
                if (line == null)
                    continue;
                IGESROW row = ParseLine(line);
                if (row.Section == IGESSection.START)
                {
                    continue;
                }
                else if (row.Section == IGESSection.GLOBAL)
                {
                    continue;
                }
                else if (row.Section == IGESSection.DIRECTORY)
                {
                    Directory.Add(row);
                }
                else if (row.Section == IGESSection.PARAMETER)
                {
                    Parameter.Add(row);
                }
                else if (row.Section == IGESSection.TERMINATE)
                {
                    break;
                }
            }

            List<IGeometry> imported_geometry = new List<IGeometry>();

            for (int i = 0; i < Directory.Count; i += 2)
            {
                IGESROW row1 = Directory[i];
                IGESROW row2 = Directory[i + 1];
                int entityType = EntityTypeFromDirectoryRow(row1);
                IIgesImportEntity? entity = null;

                if(entityType == 126)// Rational B-spline curve
                {
                    entity = new ImportedIgesRationalBSplineCurve();
                }
                else if(entityType == 128)// Rational B-spline surface
                {
                    entity = new ImportedIgesRationalBSplineSurface();
                }else
                {
                    // Unsupported entity type
                    continue;
                }

                if (entity == null)
                {
                    continue;
                }

                string[] directory_contens = new string[] { row1.Content, row2.Content };
                (var result,var start_i,var p_count) = entity.ParseDirectoryEntry(directory_contens);
                if (!result)
                {
                    throw new FormatException($"Failed to parse directory entry for entity type {entityType} at directory rows starting line {row1.LineNumber}.");
                }
                var parameter_rows = SliceParameterRow(Parameter, start_i, p_count)
                                                      .Select(p => p.Content)
                                                      .ToArray();

                var p_result = entity.ParseParameterEntity(parameter_rows);
                if(!p_result)
                {
                    throw new FormatException($"Failed to parse parameter entity for entity type {entityType} at parameter rows starting line {start_i}.");
                }
                
                imported_geometry.Add(entity.Geometry);
            }

            return imported_geometry;
        }

        static IGESROW ParseLine(string line)
        {
            if (line == null)
            {
                return new IGESROW
                {
                    Content = string.Empty,
                    Section = IGESSection.UNKNOWN,
                    LineNumber = 0
                };
            }

            string raw = line;
            // Determine section character
            char sectionChar = '\0';
            if (raw.Length >= 73)
            {
                sectionChar = raw[72];
            }
            else
            {
                throw new FormatException($"Invalid IGES line format: line too short to contain section character.");
            }

            IGESSection section = char.ToUpperInvariant(sectionChar) switch
            {
                'S' => IGESSection.START,
                'G' => IGESSection.GLOBAL,
                'D' => IGESSection.DIRECTORY,
                'P' => IGESSection.PARAMETER,
                'T' => IGESSection.TERMINATE,
                _ => IGESSection.UNKNOWN
            };
            if(section == IGESSection.UNKNOWN)
            {
                throw new FormatException($"Invalid IGES line format: unknown section character '{sectionChar}'.");
            }

            int lineNumber = -1;
            
            var numStr = raw.Substring(73, 7).Trim();
            bool result = int.TryParse(numStr, out lineNumber);
            if (!result)
            {
                throw new FormatException($"Invalid IGES line format: cannot parse line number '{numStr}'.");
            }

            string content = raw.Substring(0, 72).TrimEnd();
            

            return new IGESROW
            {
                Content = content,
                Section = section,
                LineNumber = lineNumber
            };
        }

        static int EntityTypeFromDirectoryRow(IGESROW row)
        {
            if (row.Content.Length < 8)
            {
                throw new FormatException($"Invalid IGES directory row format: content too short to contain entity type.");
            }
            string entityTypeStr = row.Content.Substring(0, 8).Trim();
            bool result = int.TryParse(entityTypeStr, out int entityType);
            if (!result)
            {
                throw new FormatException($"Invalid IGES directory row format: cannot parse entity type '{entityTypeStr}'.");
            }
            return entityType;
        }

        static List<IGESROW> SliceParameterRow(List<IGESROW> parameter_rows,int start_index,int line_count)
        {
            var sliced = parameter_rows.Skip(start_index-1).Take(line_count).ToList();
            return sliced;
            //option: scan semicolon
        }
    }

    enum IGESSection
    {
        START,
        GLOBAL,
        DIRECTORY,
        PARAMETER,
        TERMINATE,
        UNKNOWN
    }

    struct IGESROW
    {
        public string Content;//72 characters
        public IGESSection Section;
        public int LineNumber;
    }
}
