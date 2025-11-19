using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Evaluation;

namespace NurbsSharp.Operation
{
    /// <summary>
    /// (en) Operator for degree elevation and reduction of NURBS
    /// (ja) NURBSの次数昇降のためのオペレーター
    /// </summary>
    public class DegreeOperator
    {
        /// <summary>
        /// (en) Elevates the degree of the given NURBS curve by t while preserving its shape
        /// (ja) 形状を保つように与えられたNURBS曲線の次数をtだけ昇降します
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        /// <exception cref="NotImplementedException"></exception>
        public static NurbsCurve ElevateDegree(NurbsCurve curve, int t)
        {
            throw new NotImplementedException();
        }
        /// <summary>
        /// (en) Reduces the degree of the given NURBS curve by t while approximating within the specified tolerance
        /// (ja) 指定された許容誤差内で近似しながら、与えられたNURBS曲線の次数をtだけ降下させます
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="t"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        /// <exception cref="NotImplementedException"></exception>
        public static NurbsCurve ReduceDegree(NurbsCurve curve, int t, double tolerance)
        {
            throw new NotImplementedException();
        }
        
    }
}
