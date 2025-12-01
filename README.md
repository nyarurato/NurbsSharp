# NURBS Sharp
![NuGet Version](https://img.shields.io/nuget/vpre/NurbsSharp)
![Build Status](https://img.shields.io/github/actions/workflow/status/nyarurato/NurbsSharp/ci.yml?branch=master)

NURBS Sharp is a small, dependency-free NURBS (Non-Uniform Rational B-Spline) library for .NET.
It provides data structures, evaluators, topology operators, I/O and tessellation utilities implemented using only the .NET standard library.

[Japanese Readme](./README.ja.md)

<img width="800" height="800" alt="test" src="https://github.com/user-attachments/assets/b92a4e5e-70df-49f0-bcd7-7ba1030fb881" />

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Examples](#examples)
- [Documentation](#documentation)
- [Development](#development)
- [Contributing](#contributing)
- [Code of Conduct](#code-of-conduct)
- [License](#license)

## Features

- Generation and evaluation of NURBS curves and surfaces
- Topology operators: degree, knot insertion/removal, join/split
- IO helpers: simple OBJ/STL export for ONLY Mesh, IGES import / export for ONLY NURBS, simple BMP export
- Tessellation utilities for curves and surfaces
- Targets: .NET 8 and .NET Standard 2.1

## Limitations
- **NURBS Surface**: `TrimSurface` is not yet implemented.
- **Knot Vector**: Currently, only "Clamped" knot vectors are supported (multiplicity = degree + 1).
- **IGES Support**:
    - **Import**: Only supports Entity Type 126 (Rational B-spline curve) and 128 (Rational B-spline surface).
    - **Export**: Only supports NURBS and Point entities. Mesh export is not supported.
- **Mesh Export (OBJ/STL)**: Only supports Mesh objects. Direct export from NURBS is not supported (tessellation required).
- **Intersection Calculations** : Currently may be unstable depending on initial conditions.

## Installation

Install via NuGet:

```powershell
dotnet add package NurbsSharp
```

Or add the package reference to your project file.

## Quick Start

Basic usage (C#):

```csharp
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;
using NurbsSharp.IO;

// Create a NURBS surface and evaluate a point
int degreeU = 3, degreeV = 3;
var knotsU = new double[] {0,0,0,0,1,1,1,1};
var knotsV = new double[] {0,0,0,0,1,1,1,1};
var kvU = new KnotVector(knotsU);
var kvV = new KnotVector(knotsV);

// Build control points (u x v grid)
ControlPoint[][] controlPoints = new ControlPoint[4][];
controlPoints[0] = new ControlPoint[] {
    new ControlPoint(0.0, 0.0, 0.0, 1),
    new ControlPoint(1.0, 0.0, 1.0, 1),
    new ControlPoint(2.0, 0.0, 3.0, 1),
    new ControlPoint(3.0, 0.0, 3.0, 1)
};
// ... Create Control Points ...

var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);
var point = surface.GetPos(0.5, 0.5);
Console.WriteLine(point);
```

## Examples

tessellate a surface and export to STL:

```csharp
int divideN = 20;
var mesh = SurfaceTessellator.Tessellate(surface, divideN, divideN);
using (var fs = new FileStream("test_output.stl", FileMode.Create, FileAccess.Write))
{
    await STLExporter.ExportAsync(mesh, fs);
}
```

Examples Project: https://github.com/nyarurato/NurbsSharpSample  
Github Wiki: https://github.com/nyarurato/NurbsSharp/wiki/Samples  

## Documentation

API Docs: https://nyarurato.github.io/NurbsSharp/

## Development

Clone the repo and build locally:

```powershell
dotnet restore
dotnet build -c Debug
```

Run unit tests:

```powershell
dotnet test -c Debug
```

Project layout highlights:

- `src/Core` — numeric helpers and core types
- `src/Geometry` — `NurbsCurve`, `NurbsSurface`, `Mesh` etc.
- `src/Evaluation` — curve/surface evaluators
- `src/Operation` — topology operators (degree, knot, join, split)
- `src/IO` — `OBJExporter`, `STLExporter`, minimal IGES helpers

## Contributing

Contributions are welcome. Typical workflow:

1. Fork the repository
2. Create a branch with a descriptive name
3. Add tests for new behavior where appropriate
4. Open a pull request against `main` (or the target branch used in this repo)


## License

MIT License
