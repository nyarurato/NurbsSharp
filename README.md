# NURBS Sharp
![NuGet Version](https://img.shields.io/nuget/vpre/NurbsSharp)

NURBS Sharp is a NURBS (Non-Uniform Rational B-Spline) curve and surface computation library targeting .NET .  
It is implemented using only the standard library.

[Japanese Readme](./README.ja.md)

## Features

- Generation and evaluation of NURBS curves and surfaces  
- Cross-platform support (.NET 8, .NET Standard 2.1)  


## Installation
You can install NURBS Sharp via NuGet Package Manager:
```sh
dotnet add package NurbsSharp
```


## Usage

```csharp
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;
using NurbsSharp.IO;
```

```csharp
// Create a NURBS Surface
// Define degrees
int degreeU = 3;
int degreeV = 3;

// Define knot vectors
double[] knotsU = { 0, 0, 0, 0, 1, 1, 1, 1 };
double[] knotsV = { 0, 0, 0, 0, 1, 1, 1, 1 };

KnotVector knotVectorU = new KnotVector(knotsU);
KnotVector knotVectorV = new KnotVector(knotsV);

// Define control points [u][v]
ControlPoint[][] controlPoints = new ControlPoint[4][];
controlPoints[0] = new ControlPoint[] {
    // (x, y, z, weight)
    new ControlPoint(0.0, 0.0, 0.0, 1),
    new ControlPoint(1.0, 0.0, 1.0, 1),
    new ControlPoint(2.0, 0.0, 3.0, 1),
    new ControlPoint(3.0, 0.0, 3.0, 1)

};
controlPoints[1] = new ControlPoint[] {
    new ControlPoint(0.0, 1.0, 0.5, 1),
    new ControlPoint(1.0, 1.0, 1.5, 1),
    new ControlPoint(2.0, 1.0, 4.0, 1),
    new ControlPoint(3.0, 1.0, 3.0, 1)

};
controlPoints[2] = new ControlPoint[] {
    new ControlPoint(3.0, 1.0, 0.5, 1),
    new ControlPoint(3.0, 1.0, 1.5, 1),
    new ControlPoint(3.0, 1.0, 5.0, 1),
    new ControlPoint(3.0, 1.0, 6.0, 1)

};
controlPoints[3] = new ControlPoint[] {
    new ControlPoint(3.0, 2.0, 0.5, 1),
    new ControlPoint(3.0, 2.0, 1.5, 1),
    new ControlPoint(3.0, 2.0, 5.0, 1),
    new ControlPoint(3.0, 2.0, 7.0, 1)

};

// Create NURBS surface
NurbsSurface nurbsSurface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);

// Evaluate a point on the surface at (u, v) = (0.5, 0.5)
var point = nurbsSurface.GetPos(0.5, 0.5);

// It show the evaluated point
Console.WriteLine(point);

// Make a mesh from the NURBS surface
var mesh = SurfaceTessellator.Tessellate(nurbsSurface, 20, 20);

// Export the mesh to an STL file
using (FileStream fs = new FileStream("test_output.stl", FileMode.Create, FileAccess.Write))
{
    await STLExporter.ExportAsync(mesh, fs);
}


```

## Documentation
API Docs: [https://nyarurato.github.io/NurbsSharp/](https://nyarurato.github.io/NurbsSharp/)

## Development
### Build
```sh
dotnet build
```

### Test
```sh
dotnet test
```

## License

MIT License
