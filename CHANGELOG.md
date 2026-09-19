# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed
- Reduced allocations and improved performance for curve and surface position evaluation, with corresponding improvements to surface tessellation
- Reduced allocations in surface derivative evaluation and surface-surface intersection workloads
- Reduced allocations in least squares curve and surface approximation

### Fixed

### Removed

### Deprecated

## [0.4.0] - 2026-09-16

### Added
- CurveAnalyzer and SurfaceAnalyzer for geometric analysis operations
- Curve continuity evaluation APIs, ContinuityType, and CurveContinuityResult
- Curve arc-length parameterization and uniform arc-length sampling
- Closest-point query APIs on NurbsCurve and NurbsSurface
- Matrix4x4 and TransformOperator, with translation, rotation, scaling, and general transforms for curves, surfaces, and volumes
- Fast and robust ray-surface intersection APIs

### Changed
- CurveSurfaceIntersector.Intersect now uses BVH candidate detection with a marching fallback
- SurfaceSurfaceIntersector.Intersect now supports BVH and parallel execution, enabled by default
- Renamed SurfaceSurfaceIntersector.IntersectRobust to IntersectWithMarching
- Moved curve length, tangent, normal, and curvature analysis from CurveEvaluator to CurveAnalyzer
- Moved surface area, tangent, normal, and curvature analysis from SurfaceEvaluator to SurfaceAnalyzer
- Moved surface closest-point analysis from SurfaceOperator to SurfaceAnalyzer and NurbsSurface
- Parallelized large regular surface tessellation workloads and reduced adaptive tessellation allocations
- Standardized selected public method and parameter names to .NET naming conventions

### Fixed
- B-spline basis, curve, and surface derivatives at the maximum knot
- Curve continuity evaluation at connections to use exact endpoint derivatives
- Curve G1/G2 and C1/C2 classification for reparameterized and reversed connections
- Curve continuity tolerance validation and scale-independent degenerate-tangent handling
- Surface-plane intersection refinement stepping outside the surface parameter domain

### Removed
- CurveSurfaceIntersector.IntersectWithBVH; use CurveSurfaceIntersector.Intersect
- SurfaceSurfaceIntersector.IntersectRobust; use SurfaceSurfaceIntersector.IntersectWithMarching
- SurfaceOperator.FindClosestPoint overloads; use SurfaceAnalyzer or NurbsSurface closest-point APIs
- Misspelled CurveEvaluator.EvaluatTangentNormal and SurfaceEvaluator.EvaluatTangentNormal; use the analyzer APIs

### Deprecated
- Curve analysis methods on CurveEvaluator; use CurveAnalyzer
- Surface analysis methods on SurfaceEvaluator; use SurfaceAnalyzer

## [0.3.0] - 2025-12-12
### Added
- Plane
- Surface-Surface Intersector
- Surface-Plane Intersector
- Adaptive Tesselation

## [0.2.0] - 2025-12-01
### Added
- BoundingBox
- Ray
- Ray-Box Intersector
- Ray-Mesh Intersector
- Curve-Curve Intersector
- Curve-Surface Intersector
- Least Squares Approximation for curves and surfaces
- ApproximationOptions with parameterization type and clamped/free ends options
- IGES Exporter: support for Point and NURBS entities
- Surface Operator
- Knot Reduction
- Bounding Volume Hierarchy (BVH) for ray-mesh intersection
### Changed
- Add boundingbox variable to NURBS, Mesh

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
