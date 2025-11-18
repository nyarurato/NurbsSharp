using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NurbsSharp.IO.IGES
{
    /// <summary>
    /// Interface for IGES entities
    /// </summary>
    internal interface IIgesEntity
    {
        int EntityType { get; }

    }
}
