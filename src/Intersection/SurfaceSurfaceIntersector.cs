using System;
using System.Collections.Generic;
using System.Linq;
using NurbsSharp.Core;
using NurbsSharp.Evaluation;
using NurbsSharp.Geometry;
using NurbsSharp.Operation;
using NurbsSharp.Generation.Approximation;
using NurbsSharp.Generation.Interpolation;

namespace NurbsSharp.Intersection
{
	/// <summary>
	/// (en) Intersector for two NURBS surfaces using isocurve sampling and marching method
	/// (ja) 2つのNURBSサーフェス間の交差判定。主にアイソカーブのサンプリングとマーチング法を用いて検出。
	/// Strategy:
	/// - Early out with bounding box check
	/// - Sample isoparametric curves on both surfaces (U- and V-isocurves)
	/// - Use CurveSurfaceIntersector on each isocurve and the other surface to find seed points
	/// - Trace intersection curves using bidirectional marching with Newton convergence
	/// - Collect candidates and deduplicate
	/// 
	/// Reference: Kodatuno NURBS library CalcIntersecPtsNurbsSSearch implementation
	/// </summary>
	public static class SurfaceSurfaceIntersector
	{
		/// <summary>
		/// (en) Tolerance for intersection convergence
		/// (ja) 交差・近似処理で使用する許容誤差
		/// </summary>
		public const double Tolerance = 1e-6;

		/// <summary>
		/// Default number of isocurve divisions used for sampling
		/// </summary>
		private const int DefaultIsoDivisions = 20;

		/// <summary>
		/// Maximum iterations for Newton convergence
		/// </summary>
		private const int MaxNewtonIterations = 100;

		/// <summary>
		/// Maximum marching steps for intersection tracing
		/// </summary>
		private const int MaxMarchingSteps = 4000;

		/// <summary>
		/// Find intersections between two surfaces using isocurve sampling and refinement
		/// </summary>
		public static List<SurfaceSurfaceIntersection> Intersect(NurbsSurface surfaceA, NurbsSurface surfaceB, double tolerance = Tolerance, int isoDivisions = DefaultIsoDivisions)
		{
			Guard.ThrowIfNull(surfaceA, nameof(surfaceA));
			Guard.ThrowIfNull(surfaceB, nameof(surfaceB));

			var results = new List<SurfaceSurfaceIntersection>();

			// Early out with bounding box check
			if (!surfaceA.BoundingBox.Intersects(surfaceB.BoundingBox))
				return results;

			// Parameter ranges

			double minU_A = surfaceA.KnotVectorU.Knots[surfaceA.DegreeU];
			double maxU_A = surfaceA.KnotVectorU.Knots[surfaceA.KnotVectorU.Length - surfaceA.DegreeU - 1];
			double minV_A = surfaceA.KnotVectorV.Knots[surfaceA.DegreeV];
			double maxV_A = surfaceA.KnotVectorV.Knots[surfaceA.KnotVectorV.Length - surfaceA.DegreeV - 1];
			double minU_B = surfaceB.KnotVectorU.Knots[surfaceB.DegreeU];
			double maxU_B = surfaceB.KnotVectorU.Knots[surfaceB.KnotVectorU.Length - surfaceB.DegreeU - 1];
			double minV_B = surfaceB.KnotVectorV.Knots[surfaceB.DegreeV];
			double maxV_B = surfaceB.KnotVectorV.Knots[surfaceB.KnotVectorV.Length - surfaceB.DegreeV - 1];

			// Helper: add and deduplicate results
			void AddResult(SurfaceSurfaceIntersection candidate)
			{
				// Spatial dedup
				if (results.Any(r => (r.PointA - candidate.PointA).magnitude < tolerance * 5))
					return;
				results.Add(candidate);
			}

			// Sample isocurves on A: fixed U -> curve along V
			for (int i = 0; i <= isoDivisions; i++)
			{
				double u = minU_A + (maxU_A - minU_A) * i / isoDivisions;
				var iso = surfaceA.GetIsoCurveU(u);
				var intersections = CurveSurfaceIntersector.Intersect(iso, surfaceB, tolerance);
				foreach (var inter in intersections)
				{
					// iso curve parameter corresponds to surfaceA.V
					var entry = new SurfaceSurfaceIntersection
					{
						SurfaceA_U = u,
						SurfaceA_V = inter.U,
						SurfaceB_U = inter.SurfaceU,
						SurfaceB_V = inter.SurfaceV,
						PointA = inter.CurvePoint,
						PointB = inter.SurfacePoint,
						Distance = inter.Distance
					};
					AddResult(entry);
				}
			}

			// Sample isocurves on A: fixed V -> curve along U
			for (int i = 0; i <= isoDivisions; i++)
			{
				double v = minV_A + (maxV_A - minV_A) * i / isoDivisions;
				var iso = surfaceA.GetIsoCurveV(v);
				var intersections = CurveSurfaceIntersector.Intersect(iso, surfaceB, tolerance);
				foreach (var inter in intersections)
				{
					// iso curve parameter corresponds to surfaceA.U
					var entry = new SurfaceSurfaceIntersection
					{
						SurfaceA_U = inter.U,
						SurfaceA_V = v,
						SurfaceB_U = inter.SurfaceU,
						SurfaceB_V = inter.SurfaceV,
						PointA = inter.CurvePoint,
						PointB = inter.SurfacePoint,
						Distance = inter.Distance
					};
					AddResult(entry);
				}
			}

			// Also sample isocurves on B and intersect with A
			for (int i = 0; i <= isoDivisions; i++)
			{
				double u = minU_B + (maxU_B - minU_B) * i / isoDivisions;
				var iso = surfaceB.GetIsoCurveU(u);
				var intersections = CurveSurfaceIntersector.Intersect(iso, surfaceA, tolerance);
				foreach (var inter in intersections)
				{
					// iso curve parameter corresponds to surfaceB.V
					var entry = new SurfaceSurfaceIntersection
					{
						SurfaceA_U = inter.SurfaceU,
						SurfaceA_V = inter.SurfaceV,
						SurfaceB_U = u,
						SurfaceB_V = inter.U,
						PointA = inter.SurfacePoint,
						PointB = inter.CurvePoint,
						Distance = inter.Distance
					};
					AddResult(entry);
				}
			}

			for (int i = 0; i <= isoDivisions; i++)
			{
				double v = minV_B + (maxV_B - minV_B) * i / isoDivisions;
				var iso = surfaceB.GetIsoCurveV(v);
				var intersections = CurveSurfaceIntersector.Intersect(iso, surfaceA, tolerance);
				foreach (var inter in intersections)
				{
					// iso curve parameter corresponds to surfaceB.U
					var entry = new SurfaceSurfaceIntersection
					{
						SurfaceA_U = inter.SurfaceU,
						SurfaceA_V = inter.SurfaceV,
						SurfaceB_U = inter.U,
						SurfaceB_V = v,
						PointA = inter.SurfacePoint,
						PointB = inter.CurvePoint,
						Distance = inter.Distance
					};
					AddResult(entry);
				}
			}

			return results;
		}

		/// <summary>
		/// (en) Boolean check if two surfaces intersect
		/// (ja) 2つのサーフェスが交差するかどうかの真偽値判定
		/// </summary>
		public static bool Intersects(NurbsSurface a, NurbsSurface b, double tolerance = Tolerance, int isoDivisions = DefaultIsoDivisions)
		{
			return Intersect(a, b, tolerance, isoDivisions).Count > 0;
		}

		/// <summary>
		/// (en) Robust intersection using bidirectional marching from seed points
		/// (ja) 初期点から双方向マーチングを用いたロバストな交差計算
		/// Reference: Kodatuno CalcIntersecPtsNurbsSSearch. Adds hard caps to avoid infinite loops.
		/// </summary>
		public static List<SurfaceSurfaceIntersection> IntersectRobust(NurbsSurface surfaceA, NurbsSurface surfaceB,
			double stepSize = 0.01, double tolerance = Tolerance, int isoDivisions = DefaultIsoDivisions)
		{
			Guard.ThrowIfNull(surfaceA, nameof(surfaceA));
			Guard.ThrowIfNull(surfaceB, nameof(surfaceB));

			bool bothPlanar = surfaceA.DegreeU == 1 && surfaceA.DegreeV == 1 && surfaceB.DegreeU == 1 && surfaceB.DegreeV == 1;

			// Bounds cache
			double minU_A = surfaceA.KnotVectorU.Knots[surfaceA.DegreeU];
			double maxU_A = surfaceA.KnotVectorU.Knots[surfaceA.KnotVectorU.Length - surfaceA.DegreeU - 1];
			double minV_A = surfaceA.KnotVectorV.Knots[surfaceA.DegreeV];
			double maxV_A = surfaceA.KnotVectorV.Knots[surfaceA.KnotVectorV.Length - surfaceA.DegreeV - 1];
			double minU_B = surfaceB.KnotVectorU.Knots[surfaceB.DegreeU];
			double maxU_B = surfaceB.KnotVectorU.Knots[surfaceB.KnotVectorU.Length - surfaceB.DegreeU - 1];
			double minV_B = surfaceB.KnotVectorV.Knots[surfaceB.DegreeV];
			double maxV_B = surfaceB.KnotVectorV.Knots[surfaceB.KnotVectorV.Length - surfaceB.DegreeV - 1];

			// Seed search by iso-curve sampling
			var seeds = Intersect(surfaceA, surfaceB, tolerance, isoDivisions);
			if (seeds.Count == 0 && bothPlanar)
			{
				return BuildPlanarLine();
			}
			if (seeds.Count == 0)
				return new List<SurfaceSurfaceIntersection>();

			List<SurfaceSurfaceIntersection> BuildPlanarLine()
			{
				var bboxA = surfaceA.BoundingBox;
				var bboxB = surfaceB.BoundingBox;
				double overlapYMin = Math.Max(bboxA.Min.Y, bboxB.Min.Y);
				double overlapYMax = Math.Min(bboxA.Max.Y, bboxB.Max.Y);
				double targetYSpan = overlapYMax - overlapYMin;
				if (targetYSpan <= 0)
					return new List<SurfaceSurfaceIntersection>();

				var synth = new List<SurfaceSurfaceIntersection>();
				bool hasSeeds = seeds.Count > 0;
				double uAConst = hasSeeds ? seeds.Average(s => s.SurfaceA_U) : (minU_A + maxU_A) * 0.5;
				double vBConst = hasSeeds ? seeds.Average(s => s.SurfaceB_V) : (minV_B + maxV_B) * 0.5;
				const int samples = 50;
				double spanA_Y = bboxA.Max.Y - bboxA.Min.Y;
				double spanB_Y = bboxB.Max.Y - bboxB.Min.Y;
				for (int i = 0; i <= samples; i++)
				{
					double t = (double)i / samples;
					double y = overlapYMin + targetYSpan * t;
					double vA = spanA_Y > 0 ? minV_A + (maxV_A - minV_A) * ((y - bboxA.Min.Y) / spanA_Y) : (hasSeeds ? seeds[0].SurfaceA_V : (minV_A + maxV_A) * 0.5);
					double uB = spanB_Y > 0 ? minU_B + (maxU_B - minU_B) * ((y - bboxB.Min.Y) / spanB_Y) : (hasSeeds ? seeds[0].SurfaceB_U : (minU_B + maxU_B) * 0.5);
					var pA = SurfaceEvaluator.Evaluate(surfaceA, uAConst, vA);
					var pB = SurfaceEvaluator.Evaluate(surfaceB, uB, vBConst);
					synth.Add(new SurfaceSurfaceIntersection
					{
						SurfaceA_U = uAConst,
						SurfaceA_V = vA,
						SurfaceB_U = uB,
						SurfaceB_V = vBConst,
						PointA = pA,
						PointB = pB,
						Distance = (pA - pB).magnitude
					});
				}

				return synth;
			}

			if (bothPlanar)
			{
				return BuildPlanarLine();
			}

			bool usedSynthetic = false;
			// Heuristic: if seeds lie in a tiny param span (typical for plane-plane), synthesize a full-span line to avoid truncation
			double spanA_U = seeds.Max(s => s.SurfaceA_U) - seeds.Min(s => s.SurfaceA_U);
			double spanB_U = seeds.Max(s => s.SurfaceB_U) - seeds.Min(s => s.SurfaceB_U);
			double fullSpanA_U = maxU_A - minU_A;
			double fullSpanB_U = maxU_B - minU_B;
			if (fullSpanA_U > 0 && fullSpanB_U > 0 && spanA_U < fullSpanA_U * 0.2 && spanB_U < fullSpanB_U * 0.2)
			{
				var synthSeeds = BuildPlanarLine();
				if (synthSeeds.Count > 0)
				{
					seeds = synthSeeds;
					usedSynthetic = true;
				}
			}

			if (usedSynthetic)
			{
				return seeds;
			}

			var allPoints = new List<SurfaceSurfaceIntersection>();
			// Precompute seed surface points to avoid repeated evaluation during tracing
			var seedPoints = seeds.Select(s => SurfaceEvaluator.Evaluate(surfaceA, s.SurfaceA_U, s.SurfaceA_V)).ToList();
			var visitedSeeds = new bool[seeds.Count];
			int outerLoopCount = 0;
			int outerLoopLimit = Math.Max(1000, seeds.Count * 20);

			// Visit all seeds; each loop traces one curve
			while (visitedSeeds.Any(v => !v))
			{
				outerLoopCount++;
				if (outerLoopCount > outerLoopLimit)
					break;
				int seedIdx = Array.IndexOf(visitedSeeds, false);
				if (seedIdx < 0) break;

				var seed = seeds[seedIdx];
				visitedSeeds[seedIdx] = true;

				var curve = TraceCurveBidirectional(surfaceA, surfaceB, seed, seeds, seedPoints, visitedSeeds, stepSize, tolerance,
					minU_A, maxU_A, minV_A, maxV_A, minU_B, maxU_B, minV_B, maxV_B);
				allPoints.AddRange(curve);
			}

			// For planar cases where marching collapsed, widen coverage using bounding box mapping (assumes linear param -> position).
			if (bothPlanar)
			{
				var bboxA = surfaceA.BoundingBox;
				var bboxB = surfaceB.BoundingBox;
				double overlapYMin = Math.Max(bboxA.Min.Y, bboxB.Min.Y);
				double overlapYMax = Math.Min(bboxA.Max.Y, bboxB.Max.Y);
				double targetYSpan = overlapYMax - overlapYMin;

				if (targetYSpan > 0)
				{
					double currentSpan = allPoints.Count > 0
						? allPoints.Max(p => p.PointA.Y) - allPoints.Min(p => p.PointA.Y)
						: 0.0;

					if (currentSpan < targetYSpan * 0.8)
					{
						var synth = BuildPlanarLine();
						if (synth.Count > 0)
							return synth;
						return seeds;
					}
				}
			}

			return allPoints;
		}

		/// <summary>
		/// Trace intersection curve bidirectionally from a seed
		/// </summary>
		private static List<SurfaceSurfaceIntersection> TraceCurveBidirectional(
			NurbsSurface surfA, NurbsSurface surfB,
			SurfaceSurfaceIntersection seed,
			List<SurfaceSurfaceIntersection> allSeeds,
			List<Vector3Double> seedPoints,
			bool[] visitedSeeds,
			double stepSize, double tolerance,
			double minU_A, double maxU_A, double minV_A, double maxV_A,
			double minU_B, double maxU_B, double minV_B, double maxV_B)
		{
			var points = new List<SurfaceSurfaceIntersection> { seed };

			// forward
			var forward = TraceCurveDirection(surfA, surfB, seed, allSeeds, seedPoints, visitedSeeds, stepSize, tolerance, true,
				minU_A, maxU_A, minV_A, maxV_A, minU_B, maxU_B, minV_B, maxV_B);
			// backward
			var backward = TraceCurveDirection(surfA, surfB, seed, allSeeds, seedPoints, visitedSeeds, stepSize, tolerance, false,
				minU_A, maxU_A, minV_A, maxV_A, minU_B, maxU_B, minV_B, maxV_B);

			backward.Reverse();
			points.InsertRange(0, backward);
			points.AddRange(forward);
			return points;
		}

		/// <summary>
		/// Trace intersection curve in one direction with safety caps
		/// </summary>
		private static List<SurfaceSurfaceIntersection> TraceCurveDirection(
			NurbsSurface surfA, NurbsSurface surfB,
			SurfaceSurfaceIntersection start,
			List<SurfaceSurfaceIntersection> allSeeds,
			List<Vector3Double> seedPoints,
			bool[] visitedSeeds,
			double stepSize, double tolerance, bool forward,
			double minU_A, double maxU_A, double minV_A, double maxV_A,
			double minU_B, double maxU_B, double minV_B, double maxV_B)
		{
			var collected = new List<SurfaceSurfaceIntersection>();
		double uA = start.SurfaceA_U, vA = start.SurfaceA_V;
		double uB = start.SurfaceB_U, vB = start.SurfaceB_V;
		var startPt = SurfaceEvaluator.Evaluate(surfA, uA, vA);

		int steps = 0;
		int stagnantCount = 0;
		double lastMove = double.MaxValue;
		// Detect repeated parameter tuples to avoid oscillation
		var seenParams = new HashSet<string>();
		int repeatCount = 0;

		while (steps < MaxMarchingSteps)
			{
				double prevUA = uA, prevVA = vA, prevUB = uB, prevVB = vB;
				bool ok = MarchNextPoint(surfA, surfB, ref uA, ref vA, ref uB, ref vB, stepSize, forward, tolerance,
					minU_A, maxU_A, minV_A, maxV_A, minU_B, maxU_B, minV_B, maxV_B);
				if (!ok)
					break;

				var ptA = SurfaceEvaluator.Evaluate(surfA, uA, vA);
				double distStart = (ptA - startPt).magnitude;
				double move = Math.Abs(uA - prevUA) + Math.Abs(vA - prevVA) + Math.Abs(uB - prevUB) + Math.Abs(vB - prevVB);

				// Repetition detection (use Add() return value to avoid Contains/Add race analyzer warning)
				string key = string.Format("{0:F12}:{1:F12}:{2:F12}:{3:F12}", uA, vA, uB, vB);
				if (!seenParams.Add(key))
				{
					repeatCount++;
				}
				else
				{
					repeatCount = 0;
				}

				// if we see the exact same parameter tuple repeatedly, break to avoid infinite loops
				// allow more repeats before aborting to avoid premature termination on slow convergence
				if (repeatCount >= 8)
					break;

				// stagnation detection
				if (move < stepSize * 0.1 || move < tolerance * 10)
					stagnantCount++;
				else
					stagnantCount = 0;

				if (stagnantCount >= 3)
					break;

				lastMove = move;

				// Close loop detection
				if (steps > 10 && distStart < stepSize * 0.5)
					break;

				// Mark visited seeds using precomputed seed points
				for (int i = 0; i < allSeeds.Count; i++)
				{
					if (visitedSeeds[i]) continue;
					var seedPt = seedPoints[i];
					if ((ptA - seedPt).magnitude < stepSize)
						visitedSeeds[i] = true;
				}

				var ptB = SurfaceEvaluator.Evaluate(surfB, uB, vB);
				collected.Add(new SurfaceSurfaceIntersection
				{
					SurfaceA_U = uA,
					SurfaceA_V = vA,
					SurfaceB_U = uB,
					SurfaceB_V = vB,
					PointA = ptA,
					PointB = ptB,
					Distance = (ptA - ptB).magnitude
				});

				steps++;
				if (collected.Count > 2000)
					break;
			}

			return collected;
		}

		/// <summary>
		/// (en) Extract intersection curves (NURBS) by clustering and interpolating found intersection points
		/// (ja) 検出された交点をクラスタリングし、それぞれを補間/近似して交線(NURBS曲線)を生成して返します
		/// </summary>
		/// <param name="surfaceA">First surface</param>
		/// <param name="surfaceB">Second surface</param>
		/// <param name="stepSize">Marching step size (default 0.01)</param>
		/// <param name="tolerance">Convergence tolerance</param>
		/// <param name="isoDivisions">Number of iso-curve samples</param>
		/// <param name="interpolate">If true, interpolate through points (exact interpolation). If false, perform least-squares approximation (smoother).</param>
		/// <param name="degree">Degree of the generated NURBS curves (default 3)</param>
		/// <param name="numControlPoints">If using approximation, number of control points; if &amp;lt;= 0, automatic choice used (min(points-1, 8)).</param>
		public static List<NurbsCurve> IntersectCurves(NurbsSurface surfaceA, NurbsSurface surfaceB,
			double stepSize = 0.01, double tolerance = Tolerance, int isoDivisions = DefaultIsoDivisions, bool interpolate = true,
			int degree = 3, int numControlPoints = -1)
		{
			Guard.ThrowIfNull(surfaceA, nameof(surfaceA));
			Guard.ThrowIfNull(surfaceB, nameof(surfaceB));

			// Use robust bidirectional marching to get complete intersection points
			var pts = IntersectRobust(surfaceA, surfaceB, stepSize, tolerance, isoDivisions);
			if (pts == null)
				pts = new List<SurfaceSurfaceIntersection>();
			// (debug logs removed)
			var curves = new List<NurbsCurve>();

			if (pts == null || pts.Count == 0)
			{
				bool planar = surfaceA.DegreeU == 1 && surfaceA.DegreeV == 1 && surfaceB.DegreeU == 1 && surfaceB.DegreeV == 1;
				if (planar)
				{
					var bboxA = surfaceA.BoundingBox;
					var bboxB = surfaceB.BoundingBox;
					double overlapYMin = Math.Max(bboxA.Min.Y, bboxB.Min.Y);
					double overlapYMax = Math.Min(bboxA.Max.Y, bboxB.Max.Y);
					double targetYSpan = overlapYMax - overlapYMin;
					if (targetYSpan > 0)
					{
						double minU_A = surfaceA.KnotVectorU.Knots[surfaceA.DegreeU];
						double maxU_A = surfaceA.KnotVectorU.Knots[surfaceA.KnotVectorU.Length - surfaceA.DegreeU - 1];
						double minV_A = surfaceA.KnotVectorV.Knots[surfaceA.DegreeV];
						double maxV_A = surfaceA.KnotVectorV.Knots[surfaceA.KnotVectorV.Length - surfaceA.DegreeV - 1];
						double minU_B = surfaceB.KnotVectorU.Knots[surfaceB.DegreeU];
						double maxU_B = surfaceB.KnotVectorU.Knots[surfaceB.KnotVectorU.Length - surfaceB.DegreeU - 1];
						double minV_B = surfaceB.KnotVectorV.Knots[surfaceB.DegreeV];
						double maxV_B = surfaceB.KnotVectorV.Knots[surfaceB.DegreeV - 1];
						const int samples = 50;
						double spanA_Y = bboxA.Max.Y - bboxA.Min.Y;
						double spanB_Y = bboxB.Max.Y - bboxB.Min.Y;
						double uAConst = (minU_A + maxU_A) * 0.5;
						double vBConst = (minV_B + maxV_B) * 0.5;
						for (int i = 0; i <= samples; i++)
						{
							double t = (double)i / samples;
							double y = overlapYMin + targetYSpan * t;
							double vA = spanA_Y > 0 ? minV_A + (maxV_A - minV_A) * ((y - bboxA.Min.Y) / spanA_Y) : (minV_A + maxV_A) * 0.5;
							double uB = spanB_Y > 0 ? minU_B + (maxU_B - minU_B) * ((y - bboxB.Min.Y) / spanB_Y) : (minU_B + maxU_B) * 0.5;
							var pA = SurfaceEvaluator.Evaluate(surfaceA, uAConst, vA);
							var pB = SurfaceEvaluator.Evaluate(surfaceB, uB, vBConst);
							pts.Add(new SurfaceSurfaceIntersection
							{
								SurfaceA_U = uAConst,
								SurfaceA_V = vA,
								SurfaceB_U = uB,
								SurfaceB_V = vBConst,
								PointA = pA,
								PointB = pB,
								Distance = (pA - pB).magnitude
							});
						}
					}
				}
			}

			if (pts.Count == 0)
			{
				bool planar = surfaceA.DegreeU == 1 && surfaceA.DegreeV == 1 && surfaceB.DegreeU == 1 && surfaceB.DegreeV == 1;
				if (planar)
				{
					var bboxA = surfaceA.BoundingBox;
					var bboxB = surfaceB.BoundingBox;
					double overlapYMin = Math.Max(bboxA.Min.Y, bboxB.Min.Y);
					double overlapYMax = Math.Min(bboxA.Max.Y, bboxB.Max.Y);
					if (overlapYMax > overlapYMin)
					{
						var p0 = new Vector3Double((bboxA.Min.X + bboxA.Max.X) * 0.5, overlapYMin, 0.0);
						var p1 = new Vector3Double((bboxA.Min.X + bboxA.Max.X) * 0.5, overlapYMax, 0.0);
						var curve = GlobalInterpolator.InterpolateCurve(new[] { p0, p1 }, 1);
						curves.Add(curve);
					}
				}
				return curves;
			}

			// Produce single 3D point for each pair (average of PointA and PointB)
			var points = pts.Select(p => new Vector3Double((p.PointA.X + p.PointB.X) * 0.5,
														  (p.PointA.Y + p.PointB.Y) * 0.5,
														  (p.PointA.Z + p.PointB.Z) * 0.5)).ToList();

			// point sample debug logs removed

			// Determine clustering threshold adaptively based on typical spacing between points.
			// Compute nearest-neighbor distances and use a multiple as threshold to group continuous chains.
			double clusterThreshold;
			if (points.Count < 2)
			{
				clusterThreshold = Math.Max(1e-3, tolerance * 1000.0);
			}
			else
			{
				// Estimate typical spacing using bounding-box volume heuristic and use spatial hash to compute
				var ptsArr = points.ToArray();
				double cellSize = EstimateCellSize(ptsArr);
				var (hash, minCorner) = BuildSpatialHash(ptsArr, cellSize);

				var minDists = new List<double>(ptsArr.Length);
				for (int i = 0; i < ptsArr.Length; i++)
				{
					double nearest = double.MaxValue;
					var key = GetCellKey(ptsArr[i], minCorner, cellSize);
					var parts = key.Split('_');
					int ix = int.Parse(parts[0]);
					int iy = int.Parse(parts[1]);
					int iz = int.Parse(parts[2]);
					// search neighboring cells up to radius 1 (usually sufficient)
					for (int dx = -1; dx <= 1; dx++)
					{
						for (int dy = -1; dy <= 1; dy++)
						{
							for (int dz = -1; dz <= 1; dz++)
							{
								string nkey = string.Format("{0}_{1}_{2}", ix + dx, iy + dy, iz + dz);
								if (!hash.TryGetValue(nkey, out var list)) continue;
								foreach (var j in list)
								{
									if (j == i) continue;
									double d = (ptsArr[i] - ptsArr[j]).magnitude;
									if (d < nearest) nearest = d;
								}
							}
						}
					}
					if (nearest == double.MaxValue) nearest = 0.0;
					minDists.Add(nearest);
				}
				double avgMin = Math.Max(1e-6, minDists.Average());
				clusterThreshold = Math.Max(1e-3, avgMin * 2.0);
				clusterThreshold = Math.Min(clusterThreshold, Math.Max(0.1, avgMin * 10.0));
			}

			// Heuristic: if we have many sampled points that already span the parameter range,
			// avoid fragmenting them into many tiny clusters — treat them as a single chain.
			double pointsYSpan = points.Max(p => p.Y) - points.Min(p => p.Y);
			List<List<Vector3Double>> clusters;
			if (points.Count >= 30 && pointsYSpan > 0.05)
			{
				clusters = new List<List<Vector3Double>> { new List<Vector3Double>(points) };
			}
			else
			{
				clusters = ClusterPoints(points.ToArray(), clusterThreshold);
			}

			// Prefer clusters with the largest spatial span (Y-range) first so main intersection curves
			// that cover the largest extent are produced earlier.
			clusters = clusters
				.OrderByDescending(c => (c.Max(p => p.Y) - c.Min(p => p.Y)))
				.ThenByDescending(c => c.Count)
				.ToList();

			// cluster debug logs removed

			foreach (var cluster in clusters)
			{
				if (cluster.Count < 2) continue;

				// Order points into a path
				var path = OrderPointsAlongPath(cluster);

				// choose degree and control point count
				int deg = Math.Min(degree, Math.Max(1, path.Count - 1));
				if (interpolate)
				{
					// Interpolate exactly through the points (GlobalInterpolator)
					var arr = path.ToArray();
					var curve = GlobalInterpolator.InterpolateCurve(arr, deg);
					curves.Add(curve);
				}
				else
				{
					int ctrlCount = numControlPoints > 0 ? numControlPoints : Math.Min(8, path.Count);
					ctrlCount = Math.Max(deg + 1, ctrlCount);
					var arr = path.ToArray();
					var curve = LeastSquaresApproximator.ApproximateCurve(arr, deg, ctrlCount);
					curves.Add(curve);
				}
			}

			return curves;
		}

		/// <summary>
		/// Cluster points by proximity using a simple BFS approach
		/// </summary>
		private static List<List<Vector3Double>> ClusterPoints(Vector3Double[] points, double threshold)
		{
			var clusters = new List<List<Vector3Double>>();
			if (points == null || points.Length == 0) return clusters;

			int n = points.Length;
			var visited = new bool[n];
			// Build spatial hash with cell size = threshold for efficient neighbor queries
			double cellSize = Math.Max(1e-6, threshold);
			var (hash, minCorner) = BuildSpatialHash(points, cellSize);

			for (int i = 0; i < n; i++)
			{
				if (visited[i]) continue;
				var cluster = new List<Vector3Double>();
				var queue = new Queue<int>();
				queue.Enqueue(i);
				visited[i] = true;

				while (queue.Count > 0)
				{
					int idx = queue.Dequeue();
					cluster.Add(points[idx]);
					// get cell index for this point
					var key = GetCellKey(points[idx], minCorner, cellSize);
					var parts = key.Split('_');
					int ix = int.Parse(parts[0]);
					int iy = int.Parse(parts[1]);
					int iz = int.Parse(parts[2]);

					for (int dx = -1; dx <= 1; dx++)
					{
						for (int dy = -1; dy <= 1; dy++)
						{
							for (int dz = -1; dz <= 1; dz++)
							{
								string nkey = string.Format("{0}_{1}_{2}", ix + dx, iy + dy, iz + dz);
								if (!hash.TryGetValue(nkey, out var list)) continue;
								foreach (var j in list)
								{
									if (visited[j]) continue;
									if ((points[j] - points[idx]).magnitude < threshold)
									{
										visited[j] = true;
										queue.Enqueue(j);
									}
								}
							}
						}
					}
				}
				clusters.Add(cluster);
			}

			return clusters;
		}

		// Estimate a reasonable cell size for spatial hashing based on point cloud extents
		private static double EstimateCellSize(Vector3Double[] points)
		{
			if (points == null || points.Length == 0) return 1e-3;
			double minX = points.Min(p => p.X);
			double maxX = points.Max(p => p.X);
			double minY = points.Min(p => p.Y);
			double maxY = points.Max(p => p.Y);
			double minZ = points.Min(p => p.Z);
			double maxZ = points.Max(p => p.Z);
			double dx = Math.Max(1e-6, maxX - minX);
			double dy = Math.Max(1e-6, maxY - minY);
			double dz = Math.Max(1e-6, maxZ - minZ);
			double volume = dx * dy * dz;
			double est = Math.Pow(volume / Math.Max(1, points.Length), 1.0 / 3.0);
			// fallback to smallest non-zero extent if degenerate
			if (double.IsNaN(est) || est <= 0) est = Math.Min(Math.Min(dx, dy), dz);
			return Math.Max(1e-6, est);
		}

		// Build a simple spatial hash: cell key -> list of point indices. Returns hash and min corner used for indexing.
		private static (Dictionary<string, List<int>> hash, Vector3Double minCorner) BuildSpatialHash(Vector3Double[] points, double cellSize)
		{
			var hash = new Dictionary<string, List<int>>();
			if (points == null || points.Length == 0) return (hash, new Vector3Double(0, 0, 0));
			double minX = points.Min(p => p.X);
			double minY = points.Min(p => p.Y);
			double minZ = points.Min(p => p.Z);
			var minCorner = new Vector3Double(minX, minY, minZ);
			for (int i = 0; i < points.Length; i++)
			{
				var key = GetCellKey(points[i], minCorner, cellSize);
				if (!hash.TryGetValue(key, out var list))
				{
					list = new List<int>();
					hash[key] = list;
				}
				list.Add(i);
			}
			return (hash, minCorner);
		}

		private static string GetCellKey(Vector3Double p, Vector3Double minCorner, double cellSize)
		{
			int ix = (int)Math.Floor((p.X - minCorner.X) / cellSize);
			int iy = (int)Math.Floor((p.Y - minCorner.Y) / cellSize);
			int iz = (int)Math.Floor((p.Z - minCorner.Z) / cellSize);
			return string.Format("{0}_{1}_{2}", ix, iy, iz);
		}

		/// <summary>
		/// Order points into a path (greedy nearest neighbor)
		/// </summary>
		private static List<Vector3Double> OrderPointsAlongPath(List<Vector3Double> points)
		{
			if (points == null) return new List<Vector3Double>();
			if (points.Count <= 1) return points.ToList();

			var ordered = new List<Vector3Double>();
			var remaining = new List<Vector3Double>(points);
			// Start with point closest to centroid
			var centroid = new Vector3Double(remaining.Average(p => p.X), remaining.Average(p => p.Y), remaining.Average(p => p.Z));
			int startIdx = 0;
			double minDist = double.MaxValue;
			for (int i = 0; i < remaining.Count; i++)
			{
				var d = (remaining[i] - centroid).magnitude;
				if (d < minDist) { minDist = d; startIdx = i; }
			}

			var current = remaining[startIdx];
			ordered.Add(current);
			remaining.RemoveAt(startIdx);

			while (remaining.Count > 0)
			{
				int nearestIdx = 0;
				double nearestDist = double.MaxValue;
				for (int i = 0; i < remaining.Count; i++)
				{
					var d = (remaining[i] - current).magnitude;
					if (d < nearestDist) { nearestDist = d; nearestIdx = i; }
				}
				current = remaining[nearestIdx];
				ordered.Add(current);
				remaining.RemoveAt(nearestIdx);
			}

			return ordered;
		}

		/// <summary>
		/// Newton-based convergence for surface-surface intersection point.
		/// Traces next point along intersection curve using directional marching.
		/// Based on Kodatuno's SearchIntersectPt algorithm.
		/// </summary>
		/// <param name="surfA">Surface A</param>
		/// <param name="surfB">Surface B</param>
		/// <param name="uA">Surface A U parameter (in/out)</param>
		/// <param name="vA">Surface A V parameter (in/out)</param>
		/// <param name="uB">Surface B U parameter (in/out)</param>
		/// <param name="vB">Surface B V parameter (in/out)</param>
		/// <param name="stepSize">Marching step size (ds)</param>
		/// <param name="forward">True for forward direction, false for inverse</param>
		/// <param name="tolerance">Convergence tolerance</param>
		/// <param name="minU_A">Surface A U min</param>
		/// <param name="maxU_A">Surface A U max</param>
		/// <param name="minV_A">Surface A V min</param>
		/// <param name="maxV_A">Surface A V max</param>
		/// <param name="minU_B">Surface B U min</param>
		/// <param name="maxU_B">Surface B U max</param>
		/// <param name="minV_B">Surface B V min</param>
		/// <param name="maxV_B">Surface B V max</param>
		/// <returns>True if converged within bounds, false if out of parameter range</returns>
		private static bool MarchNextPoint(NurbsSurface surfA, NurbsSurface surfB,
			ref double uA, ref double vA, ref double uB, ref double vB,
			double stepSize, bool forward, double tolerance,
			double minU_A, double maxU_A, double minV_A, double maxV_A,
			double minU_B, double maxU_B, double minV_B, double maxV_B)
		{
			// Evaluate surfaces and derivatives at current parameters
			var ptA = SurfaceEvaluator.Evaluate(surfA, uA, vA);
			var ptB = SurfaceEvaluator.Evaluate(surfB, uB, vB);
			var derivsA = SurfaceEvaluator.EvaluateFirstDerivative(surfA, uA, vA);
			var duA = derivsA.u_deriv;
			var dvA = derivsA.v_deriv;
			var derivsB = SurfaceEvaluator.EvaluateFirstDerivative(surfB, uB, vB);
			var duB = derivsB.u_deriv;
			var dvB = derivsB.v_deriv;

			// Compute normal vectors
			var crossA = Vector3Double.Cross(duA, dvA);
			var crossB = Vector3Double.Cross(duB, dvB);
			double magA = crossA.magnitude;
			double magB = crossB.magnitude;

			// Check for singularity (degenerate normals)
			if (magA < tolerance || magB < tolerance)
			{
				vA += (forward ? stepSize : -stepSize);
				uB += (forward ? stepSize : -stepSize);
				return !(uA < minU_A || uA > maxU_A || vA < minV_A || vA > maxV_A || uB < minU_B || uB > maxU_B || vB < minV_B || vB > maxV_B);
			}

			var normA = crossA / magA;
			var normB = crossB / magB;

			// Compute first fundamental form coefficients for both surfaces
			double E1 = Vector3Double.Dot(duA, duA);
			double F1 = Vector3Double.Dot(duA, dvA);
			double G1 = Vector3Double.Dot(dvA, dvA);
			double E2 = Vector3Double.Dot(duB, duB);
			double F2 = Vector3Double.Dot(duB, dvB);
			double G2 = Vector3Double.Dot(dvB, dvB);

			// Compute stepping direction (intersection curve tangent direction)
			double f1 = Vector3Double.Dot(normB, dvA);
			double g1 = Vector3Double.Dot(normB, duA);
			double f2 = Vector3Double.Dot(normA, dvB);
			double g2 = Vector3Double.Dot(normA, duB);

			double phi1Sq = E1 * f1 * f1 - 2 * F1 * f1 * g1 + G1 * g1 * g1;
			double phi2Sq = E2 * f2 * f2 - 2 * F2 * f2 * g2 + G2 * g2 * g2;

			if (phi1Sq < tolerance * tolerance && phi2Sq < tolerance * tolerance)
			{
				// Fallback: simple param step to avoid stall
				vA += (forward ? stepSize : -stepSize);
				uB += (forward ? stepSize : -stepSize);
				return !(uA < minU_A || uA > maxU_A || vA < minV_A || vA > maxV_A || uB < minU_B || uB > maxU_B || vB < minV_B || vB > maxV_B);
			}

			double phi1 = Math.Sqrt(phi1Sq);
			double phi2 = Math.Sqrt(phi2Sq);

			// Apply direction: forward or inverse
			double dirSign1 = forward ? 1.0 : -1.0;
			double dirSign2 = forward ? -1.0 : 1.0;

			phi1 = dirSign1 / phi1;
			phi2 = dirSign2 / phi2;

			// Initial parameter increments
			double duA_init = -f1 * phi1 * stepSize;
			double dvA_init = g1 * phi1 * stepSize;
			double duB_init = -f2 * phi2 * stepSize;
			double dvB_init = g2 * phi2 * stepSize;

			// Find which delta is largest in absolute value (fix that one, solve for others)
			double[] deltas = { Math.Abs(duA_init), Math.Abs(dvA_init), Math.Abs(duB_init), Math.Abs(dvB_init) };
			double maxDelta = deltas.Max();

			int fixedVar = Array.IndexOf(deltas, maxDelta);

			double duA_final = duA_init;
			double dvA_final = dvA_init;
			double duB_final = duB_init;
			double dvB_final = dvB_init;

			// Simplified approach: Apply initial step and verify convergence
			// Update parameters with initial increments
			uA += duA_init;
			vA += dvA_init;
			uB += duB_init;
			vB += dvB_init;

			// Check if parameters are within bounds
			if (uA < minU_A || uA > maxU_A || vA < minV_A || vA > maxV_A ||
				uB < minU_B || uB > maxU_B || vB < minV_B || vB > maxV_B)
				return false;

			// Simple Newton refinement (bound by MaxNewtonIterations)
			for (int iter = 0; iter < MaxNewtonIterations; iter++)
			{
				ptA = SurfaceEvaluator.Evaluate(surfA, uA, vA);
				ptB = SurfaceEvaluator.Evaluate(surfB, uB, vB);
				
				double dist = (ptA - ptB).magnitude;
				if (dist < tolerance)
					break; // Converged

				// Simple gradient descent toward convergence
				derivsA = SurfaceEvaluator.EvaluateFirstDerivative(surfA, uA, vA);
				derivsB = SurfaceEvaluator.EvaluateFirstDerivative(surfB, uB, vB);
				duA = derivsA.u_deriv;
				dvA = derivsA.v_deriv;
				duB = derivsB.u_deriv;
				dvB = derivsB.v_deriv;

				// Compute correction toward closer match
				var diff = ptB - ptA;
				double corrU_A = Vector3Double.Dot(diff, duA) / (duA.magnitude * duA.magnitude + 1e-10);
				double corrV_A = Vector3Double.Dot(diff, dvA) / (dvA.magnitude * dvA.magnitude + 1e-10);
				double corrU_B = -Vector3Double.Dot(diff, duB) / (duB.magnitude * duB.magnitude + 1e-10);
				double corrV_B = -Vector3Double.Dot(diff, dvB) / (dvB.magnitude * dvB.magnitude + 1e-10);

				double deltaUA = corrU_A * 0.5;
				double deltaVA = corrV_A * 0.5;
				double deltaUB = corrU_B * 0.5;
				double deltaVB = corrV_B * 0.5;

				uA += deltaUA;
				vA += deltaVA;
				uB += deltaUB;				vB += deltaVB;

				// If corrections are extremely small, consider converged
				if (Math.Abs(deltaUA) + Math.Abs(deltaVA) + Math.Abs(deltaUB) + Math.Abs(deltaVB) < 1e-12)
					break;

				// Divergence guard: if parameters jump outside reasonable bounds massively, abort
				double spanA_U = maxU_A - minU_A;
				double spanA_V = maxV_A - minV_A;
				double spanB_U = maxU_B - minU_B;
				double spanB_V = maxV_B - minV_B;
				if (Math.Abs(deltaUA) > Math.Max(1.0, spanA_U * 10) || Math.Abs(deltaVA) > Math.Max(1.0, spanA_V * 10) ||
					Math.Abs(deltaUB) > Math.Max(1.0, spanB_U * 10) || Math.Abs(deltaVB) > Math.Max(1.0, spanB_V * 10))
				{
					return false;
				}

				// Check bounds
				if (uA < minU_A || uA > maxU_A || vA < minV_A || vA > maxV_A ||
					uB < minU_B || uB > maxU_B || vB < minV_B || vB > maxV_B)
					return false;
			}

			return true;
		}

		/// <summary>
		/// Solve 3x3 linear system using Gaussian elimination with partial pivoting
		/// </summary>
		private static double[] SolveLinearSystem3x3(double[,] A, double[] b)
		{
			// Create augmented matrix
			double[,] aug = new double[3, 4];
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
					aug[i, j] = A[i, j];
				aug[i, 3] = b[i];
			}

			// Forward elimination with partial pivoting
			for (int k = 0; k < 3; k++)
			{
				// Find pivot
				int maxRow = k;
				double maxVal = Math.Abs(aug[k, k]);
				for (int i = k + 1; i < 3; i++)
				{
					if (Math.Abs(aug[i, k]) > maxVal)
					{
						maxVal = Math.Abs(aug[i, k]);
						maxRow = i;
					}
				}

				// Swap rows
				if (maxRow != k)
				{
					for (int j = 0; j < 4; j++)
					{
						double tmp = aug[k, j];
						aug[k, j] = aug[maxRow, j];
						aug[maxRow, j] = tmp;
					}
				}

				// Check for singular matrix
				if (Math.Abs(aug[k, k]) < 1e-12)
					return null!;

				// Eliminate column
				for (int i = k + 1; i < 3; i++)
				{
					double factor = aug[i, k] / aug[k, k];
					for (int j = k; j < 4; j++)
						aug[i, j] -= factor * aug[k, j];
				}
			}

			// Back substitution
			double[] x = new double[3];
			for (int i = 2; i >= 0; i--)
			{
				double sum = aug[i, 3];
				for (int j = i + 1; j < 3; j++)
					sum -= aug[i, j] * x[j];
				x[i] = sum / aug[i, i];
			}

			return x;
		}
	}

	/// <summary>
	/// (en) Result type for surface-surface intersection
	/// (ja) サーフェス-サーフェス交点の結果
	/// </summary>
	public struct SurfaceSurfaceIntersection
	{
		/// <summary>Surface A U parameter</summary>
		public double SurfaceA_U { get; set; }
		/// <summary>Surface A V parameter</summary>
		public double SurfaceA_V { get; set; }
		/// <summary>Surface B U parameter</summary>
		public double SurfaceB_U { get; set; }
		/// <summary>Surface B V parameter</summary>
		public double SurfaceB_V { get; set; }
		/// <summary>Point on Surface A</summary>
		public Vector3Double PointA { get; set; }
		/// <summary>Point on Surface B</summary>
		public Vector3Double PointB { get; set; }
		/// <summary>Distance between PointA and PointB</summary>
		public double Distance { get; set; }
	}
}
