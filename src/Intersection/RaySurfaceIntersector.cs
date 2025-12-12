using System;
using System.Collections.Generic;
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;

namespace NurbsSharp.Intersection
{
    /// <summary>
    /// (en) Ray - NURBS surface intersection
    /// (ja) レイとNURBSサーフェスの交差判定
    /// </summary>
    public static class RaySurfaceIntersector
    {
        /// <summary>
        /// Find first intersection between a ray and a NURBS surface by tessellating the surface into a mesh and delegating to RayMeshIntersector.
        /// Fast but approximate method suitable for rendering and preview.
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="surface">Surface to test against</param>
        /// <param name="intersection">Output intersection result (if any)</param>
        /// <param name="numPointsU">Tessellation resolution along U</param>
        /// <param name="numPointsV">Tessellation resolution along V</param>
        /// <param name="bvh">Optional prebuilt mesh BVH to reuse (if null a BVH will be built internally)</param>
        /// <returns>True if an intersection was found</returns>
        public static bool IntersectsFast(Ray ray, NurbsSurface surface, out RaySurfaceIntersection intersection, int numPointsU = 30, int numPointsV = 30, MeshBVHNode? bvh = null)
        {
            intersection = default;

            Guard.ThrowIfNull(surface, nameof(surface));

            // Empty surface early out
            if (surface.ControlPoints == null || surface.ControlPoints.Length == 0)
                return false;

            // Bounding box early-out
            if (!RayBoxIntersector.Intersects(ray, surface.BoundingBox))
                return false;

            // Tessellate surface
            var mesh = SurfaceTessellator.Tessellate(surface, numPointsU, numPointsV);

            if (mesh == null || mesh.Indexes.Length == 0)
                return false;

            // Delegate to RayMeshIntersector
            if (RayMeshIntersector.Intersects(ray, mesh, out var meshHit, bvh))
            {
                // Map triangle barycentric back to surface (u,v) param space
                var (uParam, vParam) = MapTriangleBarycentricToSurfaceParams(mesh, surface, meshHit.TriangleIndex, meshHit.U, meshHit.V, numPointsU, numPointsV);

                intersection = new RaySurfaceIntersection(meshHit.T, uParam, vParam, meshHit.Point, meshHit.TriangleIndex);
                return true;
            }

            return false;
        }

        /// <summary>
        /// Find all intersections between a ray and a NURBS surface (approximate via tessellation).
        /// Fast but approximate method suitable for rendering and preview.
        /// </summary>
        public static List<RaySurfaceIntersection> IntersectAllFast(Ray ray, NurbsSurface surface, int numPointsU = 30, int numPointsV = 30, MeshBVHNode? bvh = null)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            var result = new List<RaySurfaceIntersection>();

            if (!RayBoxIntersector.Intersects(ray, surface.BoundingBox))
                return result;

            var mesh = SurfaceTessellator.Tessellate(surface, numPointsU, numPointsV);
            if (mesh == null || mesh.Indexes.Length == 0)
                return result;

            var meshHits = RayMeshIntersector.IntersectAll(ray, mesh, bvh);
            foreach (var mh in meshHits)
            {
                var (uParam, vParam) = MapTriangleBarycentricToSurfaceParams(mesh, surface, mh.TriangleIndex, mh.U, mh.V, numPointsU, numPointsV);
                result.Add(new RaySurfaceIntersection(mh.T, uParam, vParam, mh.Point, mh.TriangleIndex));
            }

            return result;
        }

        /// <summary>
        /// Quick boolean check whether any intersection exists (uses tessellation).
        /// Fast but approximate method suitable for rendering and preview.
        /// </summary>
        public static bool IntersectsAnyFast(Ray ray, NurbsSurface surface, int numPointsU = 30, int numPointsV = 30, MeshBVHNode? bvh = null)
        {
            Guard.ThrowIfNull(surface, nameof(surface));

            if (!RayBoxIntersector.Intersects(ray, surface.BoundingBox))
                return false;

            var mesh = SurfaceTessellator.Tessellate(surface, numPointsU, numPointsV);
            if (mesh == null || mesh.Indexes.Length == 0)
                return false;

            return RayMeshIntersector.IntersectsAny(ray, mesh, bvh);
        }

        /// <summary>
        /// Find first intersection between a ray and a NURBS surface using Newton-Raphson iteration.
        /// High-precision method suitable for CAD/CAM and accurate geometric computations.
        /// </summary>
        /// <param name="ray">Ray to test</param>
        /// <param name="surface">Surface to test against</param>
        /// <param name="intersection">Output intersection result (if converged)</param>
        /// <param name="tolerance">Convergence tolerance (default: 1e-6)</param>
        /// <param name="maxIterations">Maximum Newton iterations (default: 50)</param>
        /// <param name="numSamples">Initial sampling resolution for seed points (default: 10)</param>
        /// <returns>True if an intersection was found</returns>
        public static bool IntersectsRobust(Ray ray, NurbsSurface surface, out RaySurfaceIntersection intersection, double tolerance = 1e-6, int maxIterations = 50, int numSamples = 10)
        {
            intersection = default;
            Guard.ThrowIfNull(surface, nameof(surface));

            if (!RayBoxIntersector.Intersects(ray, surface.BoundingBox))
                return false;

            // Get parameter domain
            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

            // Sample surface to find good initial seeds
            var seeds = FindInitialSeeds(ray, surface, uMin, uMax, vMin, vMax, numSamples);

            RaySurfaceIntersection? bestHit = null;
            double bestT = double.MaxValue;

            foreach (var (u0, v0, t0) in seeds)
            {
                if (NewtonRaphsonRaySurface(ray, surface, u0, v0, t0, tolerance, maxIterations, out double u, out double v, out double t))
                {
                    // Verify solution is within parameter domain
                    if (u >= uMin && u <= uMax && v >= vMin && v <= vMax && t > 0)
                    {
                        var point = Evaluation.SurfaceEvaluator.Evaluate(surface, u, v);
                        var rayPoint = ray.PointAt(t);
                        double dist = (point - rayPoint).magnitude;

                        // Verify the intersection is valid
                        if (dist < tolerance * 10 && t < bestT)
                        {
                            bestT = t;
                            bestHit = new RaySurfaceIntersection(t, u, v, point, -1);
                        }
                    }
                }
            }

            if (bestHit.HasValue)
            {
                intersection = bestHit.Value;
                return true;
            }

            return false;
        }

        /// <summary>
        /// Find all intersections between a ray and a NURBS surface using Newton-Raphson iteration.
        /// High-precision method suitable for CAD/CAM and accurate geometric computations.
        /// </summary>
        public static List<RaySurfaceIntersection> IntersectAllRobust(Ray ray, NurbsSurface surface, double tolerance = 1e-6, int maxIterations = 50, int numSamples = 20)
        {
            var result = new List<RaySurfaceIntersection>();
            Guard.ThrowIfNull(surface, nameof(surface));

            if (!RayBoxIntersector.Intersects(ray, surface.BoundingBox))
                return result;

            double uMin = surface.KnotVectorU.Knots[surface.DegreeU];
            double uMax = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
            double vMin = surface.KnotVectorV.Knots[surface.DegreeV];
            double vMax = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

            var seeds = FindInitialSeeds(ray, surface, uMin, uMax, vMin, vMax, numSamples);
            var foundHits = new List<RaySurfaceIntersection>();

            foreach (var (u0, v0, t0) in seeds)
            {
                if (NewtonRaphsonRaySurface(ray, surface, u0, v0, t0, tolerance, maxIterations, out double u, out double v, out double t))
                {
                    if (u >= uMin && u <= uMax && v >= vMin && v <= vMax && t > 0)
                    {
                        var point = Evaluation.SurfaceEvaluator.Evaluate(surface, u, v);
                        var rayPoint = ray.PointAt(t);
                        double dist = (point - rayPoint).magnitude;

                        if (dist < tolerance * 10)
                        {
                            var hit = new RaySurfaceIntersection(t, u, v, point, -1);
                            
                            // Check for duplicates (same intersection found from different seeds)
                            bool isDuplicate = false;
                            foreach (var existing in foundHits)
                            {
                                if (Math.Abs(existing.T - t) < tolerance * 10 &&
                                    Math.Abs(existing.U - u) < tolerance &&
                                    Math.Abs(existing.V - v) < tolerance)
                                {
                                    isDuplicate = true;
                                    break;
                                }
                            }

                            if (!isDuplicate)
                            {
                                foundHits.Add(hit);
                            }
                        }
                    }
                }
            }

            // Sort by ray parameter t
            foundHits.Sort((a, b) => a.T.CompareTo(b.T));
            return foundHits;
        }

        /// <summary>
        /// Newton-Raphson solver for ray-surface intersection.
        /// Solves: S(u,v) - (O + t*D) = 0 for u, v, t
        /// </summary>
        private static bool NewtonRaphsonRaySurface(Ray ray, NurbsSurface surface, double u0, double v0, double t0, double tolerance, int maxIterations, out double u, out double v, out double t)
        {
            u = u0;
            v = v0;
            t = t0;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // Evaluate surface position and derivatives
                var S = Evaluation.SurfaceEvaluator.Evaluate(surface, u, v);
                var (Su, Sv) = Evaluation.SurfaceEvaluator.EvaluateFirstDerivative(surface, u, v);

                // Ray point
                var R = ray.PointAt(t);

                // Residual: f = S(u,v) - R(t)
                var f = S - R;
                double residual = f.magnitude;

                if (residual < tolerance)
                    return true;

                // Jacobian matrix:
                // [ Su_x  Sv_x  -D_x ]
                // [ Su_y  Sv_y  -D_y ]
                // [ Su_z  Sv_z  -D_z ]
                double[,] J = new double[3, 3] {
                    { Su.X, Sv.X, -ray.Direction.X },
                    { Su.Y, Sv.Y, -ray.Direction.Y },
                    { Su.Z, Sv.Z, -ray.Direction.Z }
                };

                // Solve J * delta = -f using Cramer's rule (3x3 system)
                double[] rhs = new double[] { -f.X, -f.Y, -f.Z };
                if (!Solve3x3(J, rhs, out double du, out double dv, out double dt))
                    return false; // Singular matrix

                // Update with damping for stability
                double damping = 1.0;
                if (iter > 10) damping = 0.5; // Reduce step size if slow to converge

                u += damping * du;
                v += damping * dv;
                t += damping * dt;

                // Check for divergence
                if (Math.Abs(du) > 100 || Math.Abs(dv) > 100 || Math.Abs(dt) > 100)
                    return false;
            }

            return false; // Did not converge
        }

        /// <summary>
        /// Solve 3x3 linear system using Cramer's rule
        /// </summary>
        private static bool Solve3x3(double[,] A, double[] b, out double x, out double y, out double z)
        {
            x = y = z = 0;

            // Calculate determinant
            double det = A[0, 0] * (A[1, 1] * A[2, 2] - A[1, 2] * A[2, 1])
                       - A[0, 1] * (A[1, 0] * A[2, 2] - A[1, 2] * A[2, 0])
                       + A[0, 2] * (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]);

            if (Math.Abs(det) < 1e-12)
                return false; // Singular

            // Cramer's rule
            double detX = b[0] * (A[1, 1] * A[2, 2] - A[1, 2] * A[2, 1])
                        - A[0, 1] * (b[1] * A[2, 2] - A[1, 2] * b[2])
                        + A[0, 2] * (b[1] * A[2, 1] - A[1, 1] * b[2]);

            double detY = A[0, 0] * (b[1] * A[2, 2] - A[1, 2] * b[2])
                        - b[0] * (A[1, 0] * A[2, 2] - A[1, 2] * A[2, 0])
                        + A[0, 2] * (A[1, 0] * b[2] - b[1] * A[2, 0]);

            double detZ = A[0, 0] * (A[1, 1] * b[2] - b[1] * A[2, 1])
                        - A[0, 1] * (A[1, 0] * b[2] - b[1] * A[2, 0])
                        + b[0] * (A[1, 0] * A[2, 1] - A[1, 1] * A[2, 0]);

            x = detX / det;
            y = detY / det;
            z = detZ / det;
            return true;
        }

        /// <summary>
        /// Find initial seed points by sampling the surface and finding closest approaches to the ray
        /// </summary>
        private static List<(double u, double v, double t)> FindInitialSeeds(Ray ray, NurbsSurface surface, double uMin, double uMax, double vMin, double vMax, int numSamples)
        {
            var seeds = new List<(double u, double v, double t, double dist)>();

            for (int i = 0; i <= numSamples; i++)
            {
                double u = uMin + (uMax - uMin) * i / numSamples;
                for (int j = 0; j <= numSamples; j++)
                {
                    double v = vMin + (vMax - vMin) * j / numSamples;
                    var point = Evaluation.SurfaceEvaluator.Evaluate(surface, u, v);

                    // Find closest point on ray to this surface point
                    var toPoint = point - ray.Origin;
                    double t = Vector3Double.Dot(toPoint, ray.Direction); // Project onto ray direction (assuming normalized)

                    if (t > 0)
                    {
                        var rayPoint = ray.PointAt(t);
                        double dist = (point - rayPoint).magnitude;
                        seeds.Add((u, v, t, dist));
                    }
                }
            }

            // Sort by distance and return best candidates
            seeds.Sort((a, b) => a.dist.CompareTo(b.dist));
            int numSeeds = Math.Min(seeds.Count, Math.Max(5, numSamples / 2)); // Return top candidates
            var result = new List<(double u, double v, double t)>();
            for (int i = 0; i < numSeeds && i < seeds.Count; i++)
            {
                result.Add((seeds[i].u, seeds[i].v, seeds[i].t));
            }
            return result;
        }

        private static (double u, double v) MapTriangleBarycentricToSurfaceParams(Geometry.Mesh mesh, NurbsSurface surface, int triangleIndex, double baryU, double baryV, int numPointsU, int numPointsV)
        {
            // Map mesh vertex indices to (i,j) grid coordinates and compute (u,v) for each vertex
            int idx0 = mesh.Indexes[triangleIndex * 3 + 0];
            int idx1 = mesh.Indexes[triangleIndex * 3 + 1];
            int idx2 = mesh.Indexes[triangleIndex * 3 + 2];

            void VertexIndexToUV(int idx, out double u, out double v)
            {
                int i = idx / numPointsV; // row
                int j = idx % numPointsV; // column

                double uStart = surface.KnotVectorU.Knots[surface.DegreeU];
                double uEnd = surface.KnotVectorU.Knots[surface.KnotVectorU.Knots.Length - surface.DegreeU - 1];
                double vStart = surface.KnotVectorV.Knots[surface.DegreeV];
                double vEnd = surface.KnotVectorV.Knots[surface.KnotVectorV.Knots.Length - surface.DegreeV - 1];

                if (numPointsU == 1) u = uStart;
                else u = uStart + (uEnd - uStart) * i / (numPointsU - 1);

                if (numPointsV == 1) v = vStart;
                else v = vStart + (vEnd - vStart) * j / (numPointsV - 1);
            }

            VertexIndexToUV(idx0, out var u0, out var v0);
            VertexIndexToUV(idx1, out var u1, out var v1);
            VertexIndexToUV(idx2, out var u2, out var v2);

            double w0 = 1.0 - baryU - baryV;
            double w1 = baryU;
            double w2 = baryV;

            double uRes = w0 * u0 + w1 * u1 + w2 * u2;
            double vRes = w0 * v0 + w1 * v1 + w2 * v2;

            return (uRes, vRes);
        }
    }

    /// <summary>
    /// (en) Ray-Surface intersection result (approximated via tessellation)
    /// (ja) レイ-サーフェス交差結果（テッセレーションによる近似）
    /// </summary>
    public struct RaySurfaceIntersection(double t, double u, double v, Vector3Double point, int triangleIndex)
    {
        public double T { get; set; } = t;
        public double U { get; set; } = u;
        public double V { get; set; } = v;
        public Vector3Double Point { get; set; } = point;
        public int TriangleIndex { get; set; } = triangleIndex;
    }
}
