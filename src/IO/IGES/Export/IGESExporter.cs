using NurbsSharp.Core;
using NurbsSharp.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace NurbsSharp.IO.IGES
{
    //reference: https://wiki.eclipse.org/IGES_file_Specification
    //TODO; Support Mesh Export

    /// <summary>
    /// IGES Exporter
    /// Only supports NURBS Curve and Surface export
    /// </summary>
    public class IGESExporter
    {
        /// <summary>
        /// (en) Export NURBS surface to IGES format
        /// (ja) NURBSサーフェスをIGES形式でエクスポートする
        /// </summary>
        /// <param name="surfaces"></param>
        /// <param name="stream"></param>
        /// <param name="fileName"></param>
        /// <param name="author"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static async Task<bool> ExportAsync(List<NurbsSurface> surfaces, Stream stream, string fileName = "model.igs", string author = "NurbsSharp")
        {
            Guard.ThrowIfNull(surfaces, nameof(surfaces));
            Guard.ThrowIfNull(stream, nameof(stream));

            using var writer = new StreamWriter(stream, Encoding.ASCII, 1024, leaveOpen: true);
            var iges = new IgesWriter(writer);
            int countS = 0, countG = 0, countD = 0, countP = 0;


            // 1) Start Section
            countS = await WriteStartSection(iges, fileName, author);

            // 2) Global Section
            countG = await WriteGlobalSection(iges);

            // 3) Create Export Entities
            List<IIgesExportEntity> entities = [];
            foreach (var surface in surfaces)
            {
                entities.Add(new IgesRationalBSplineSurface(surface));
            }

            //3.5 ) Prepare Directory / Parameter Section
            foreach (var e in entities)
            {
                e.GenerateParameterData();
            }

            // 4) Directory Section
            countD = await WriteDirectorySection(iges, entities);
            // 5) Parameter Section
            countP = await WriteParameterSectionAsync(iges, entities);

            // 6) Terminate Section
            WriteTerminateSection(iges, countS, countG, countD, countP);

            await writer.FlushAsync();
            return true;
        }


        /// <summary>
        /// (en) Export NURBS curve to IGES format
        /// (ja) NURBS曲線をIGES形式でエクスポートする
        /// </summary>
        /// <param name="curves"></param>
        /// <param name="stream"></param>
        /// <param name="fileName"></param>
        /// <param name="author"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public static async Task<bool> ExportAsync(List<NurbsCurve> curves, Stream stream, string fileName = "model.igs", string author = "NurbsSharp")
        {
            Guard.ThrowIfNull(curves, nameof(curves));
            Guard.ThrowIfNull(stream, nameof(stream));

            using var writer = new StreamWriter(stream, Encoding.ASCII, 1024, leaveOpen: true);
            var iges = new IgesWriter(writer);
            int countS = 0, countG = 0, countD = 0, countP = 0;

            countS = await WriteStartSection(iges, fileName, author);
            countG = await WriteGlobalSection(iges);

            var entities = new List<IIgesExportEntity>();
            foreach(var curve in curves)
            {
                entities.Add(new IgesRationalBSplineCurve(curve));
            };

            //Prepare Directory / Parameter Section (Estimate Parameter Line Count)
            foreach (var e in entities)
            {
                e.GenerateParameterData();
            }

            countD = await WriteDirectorySection(iges, entities);
            countP = await WriteParameterSectionAsync(iges, entities);

            WriteTerminateSection(iges, countS, countG, countD, countP);

            await writer.FlushAsync();
            return true;
        }

        /// <summary>
        /// Export Mesh (Not Supported Yet) 
        /// </summary>
        [Obsolete("IGES export:Export Mesh not support yet")]
        public static Task<bool> ExportAsync(Mesh mesh, Stream stream)
        {
            //TODO : Implement Mesh Export
            throw new NotSupportedException("IGES export:Export Mesh not support yet");
        }

        private static async Task<int> WriteStartSection(IgesWriter w, string fileName, string author)
        {

            // S Section
            return await w.WriteRecordForS_G_TAsync("Start section - IGES export by " + author + " - " + fileName, 'S');
        }

        private static async Task<int> WriteGlobalSection(IgesWriter w)
        {
            int countG = 0;
            // G Section
            countG += await w.WriteRecordForS_G_TAsync("1H,,1H;,10HNURBSSHARP,6HCSharp,4HRecv,4HData,16,38,06,38,13,", 'G');//Parm1-11
            countG += await w.WriteRecordForS_G_TAsync("10HNURBSSHARP,1.0,2,2HMM,1,,15H20250101.000000,1E-8,1E10,", 'G');//Parm12-20
            countG += await w.WriteRecordForS_G_TAsync("10HNURBSSHARP,6HCSharp,6,0;", 'G');//Parm21-24
            return countG;
        }

        private static async Task<int> WriteDirectorySection(IgesWriter w, IEnumerable<IIgesExportEntity> entities)
        {
            // D section
            int idx = 1;
            int paramPointerIndex = 1;
            foreach (var e in entities)
            {
                e.SetDirectoryPointerToParameterString(idx);
                foreach (var d in e.GenerateDirectoryEntry(paramPointerIndex, e.ParameterLineCount))
                {
                    
                    await w.WriteRecordForDAsync(d, idx);
                    idx++;
                }
                paramPointerIndex += e.ParameterLineCount;
            }
            return idx - 1;
        }

        private static async Task<int> WriteParameterSectionAsync(IgesWriter w, IEnumerable<IIgesExportEntity> entities)
        {
            // P Section
            // Data Column 1-72
            int idx = 1;
            foreach (var e in entities)
            {
                foreach (var paramData in e.ParameterData)
                {
                    await w.WriteRecordForPAsync(paramData, 'P');
                    idx++;
                }
            }
            return idx - 1;
        }

        private static async void WriteTerminateSection(IgesWriter w,int countS,int countG,int countD,int countP)
        {
            var record = $"S{countS:D7}G{countG:D7}D{countD:D7}P{countP:D7}";
            await w.WriteRecordForS_G_TAsync(record, 'T');
        }



        /// <summary>
        /// (en) Helper class to manage and write IGES fixed-length records (80 characters: 1-72 data, 73 section code (char), 74-80 seq number (7 chars))
        /// (ja)IGES 固定長レコード（80文字: 1-72 データ、73 section code (char)、74-80 seq number (7 chars)）を管理して書き出すヘルパー
        /// </summary>
        private class IgesWriter
        {
            private readonly StreamWriter _w;
            private readonly Dictionary<char, int> _counters;

            public IgesWriter(StreamWriter w)
            {
                _w = w ?? throw new ArgumentNullException(nameof(w));
                _counters = new Dictionary<char, int>
                {
                    ['S'] = 1,
                    ['G'] = 1,
                    ['D'] = 1,
                    ['P'] = 1,
                    ['T'] = 1
                };
            }

            /// <summary>
            /// (en) Split a string into chunks of maximum 72 characters for IGES record data.
            /// (ja) IGESレコードデータ用に最大72文字のチャンクに文字列を分割する
            /// </summary>
            /// <param name="s"></param>
            /// <returns></returns>
            private static IEnumerable<string> Split72(string s)
            {

                if (string.IsNullOrEmpty(s))
                {
                    return [string.Empty];
                }

                int chunkCount = (s.Length + 71) / 72; //Ceiling
                return Enumerable.Range(0, chunkCount)
                                 .Select(i =>
                                 {
                                     int start = i * 72;
                                     int len = Math.Min(72, s.Length - start);
                                     return s.Substring(start, len);
                                 });
            }


            private string FormatRecordLine(string data72, char section)
            {
                if (!_counters.ContainsKey(section)) _counters[section] = 1;
                int seq = _counters[section]++;
                string dataPadded = data72.Length > 72 ? data72.Substring(0, 72) : data72.PadRight(72, ' ');
                string seqStr = seq.ToString().PadLeft(7, ' ');
                return dataPadded + section + seqStr;
            }

            /// <summary>
            /// (en) Write IGES record lines for S, G, or T sections, splitting data into 72-character chunks as needed.
            /// (ja) S、G、Tセクション用のIGESレコード行を書き出し、必要に応じてデータを72文字チャンクに分割する
            /// </summary>
            /// <param name="data"></param>
            /// <param name="section"></param>

            public async Task<int> WriteRecordForS_G_TAsync(string data, char section)
            {
                int columnCount = 0;
                foreach (var chunk in Split72(data))
                {
                    columnCount++;
                    string line = FormatRecordLine(chunk, section);
                    await _w.WriteLineAsync(line);
                }
                return columnCount;
            }

            /// <summary>
            /// (en) Write IGES record line for D section
            /// (ja) Dセクション用のIGESレコード行を書き出す
            /// </summary>
            /// <param name="data"></param>
            /// <param name="index"></param>
            /// <returns></returns>
            public async Task<int> WriteRecordForDAsync(string data,int index)
            {
                await _w.WriteLineAsync(data+"D"+ index.ToString().PadLeft(7, ' '));
                return 1;
            }

            /// <summary>
            /// (en) Write IGES record lines for P section, splitting data into 72-character chunks as needed.
            /// (ja) Pセクション用のIGESレコード行を書き出し、必要に応じてデータを72文字チャンクに分割する
            /// </summary>
            /// <param name="fullData"></param>
            /// <param name="section"></param>
            /// <returns></returns>
            public async Task WriteRecordForPAsync(string fullData, char section)
            {
                // For P section
                foreach (var chunk in Split72(fullData))
                {
                    string line = FormatRecordLine(chunk, section);
                    await _w.WriteLineAsync(line);
                }
            }
        }

    }
}
