# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- BoundingBox
- Ray
- Ray-Box Intersector
- Ray-Mesh Intersector
### Changed
- Add boundingbox variable to NURBS, Mesh

### Fixed

### Removed

### Deprecated

## [0.1.1] - 2025-11-25
### Added
- JoinCurves method in JoinOperator
- JoinSurfaces method in JoinOperator (U and V direction joining)
- SplitSurface method in SplitOperator (U and V direction splitting)
- Simple BMP Exporter
- ElevateDegree for surfaces in DegreeOperator
- ReduceDegree for surfaces in DegreeOperator
- InsertKnot for surfaces in KnotOperator

### Changed
- Vector3Double, Vector4Double class -> readonly struct
- Export curve list as IGES file

## [0.1.0] - 2025-11-21

### Added
- NURBS curve and surface evaluation (position, derivatives, curvature)
- Topology operators (degree elevation/reduction, knot insertion/removal)
- I/O support (OBJ, STL, IGES)
- Tessellation for curves and surfaces
- Primitive generators (line, circle, box, sphere, cylinder)
- Global interpolation with multiple parameterization methods