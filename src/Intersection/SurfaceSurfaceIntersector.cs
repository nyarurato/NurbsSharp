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
    // TODO: Newton method
	/// <summary>
	/// (en) Intersector for two NURBS surfaces using isocurve sampling and CurveSurfaceIntersector refinement
	/// (ja) 2つのNURBSサーフェス間の交差判定。主にアイソカーブのサンプリングとCurveSurfaceIntersectorを用いて検出。
	/// Strategy:
	/// - Early out with bounding box check
	/// - Sample isoparametric curves on both surfaces (U- and V-isocurves)
	/// - Use CurveSurfaceIntersector on each isocurve and the other surface
	/// - Collect candidates and deduplicate
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
		/// (en) Extract intersection curves (NURBS) by clustering and interpolating found intersection points
		/// (ja) 検出された交点をクラスタリングし、それぞれを補間/近似して交線（NURBS曲線）を生成して返します
		/// </summary>
		/// <param name="surfaceA">First surface</param>
		/// <param name="surfaceB">Second surface</param>
		/// <param name="tolerance">Convergence tolerance</param>
		/// <param name="isoDivisions">Number of iso-curve samples</param>
		/// <param name="interpolate">If true, interpolate through points (exact interpolation). If false, perform least-squares approximation (smoother).</param>
		/// <param name="degree">Degree of the generated NURBS curves (default 3)</param>
		/// <param name="numControlPoints">If using approximation, number of control points; if &amp;lt;= 0, automatic choice used (min(points-1, 8)).</param>
		public static List<NurbsCurve> IntersectCurves(NurbsSurface surfaceA, NurbsSurface surfaceB,
			double tolerance = Tolerance, int isoDivisions = DefaultIsoDivisions, bool interpolate = true,
			int degree = 3, int numControlPoints = -1)
		{
			Guard.ThrowIfNull(surfaceA, nameof(surfaceA));
			Guard.ThrowIfNull(surfaceB, nameof(surfaceB));

			var pts = Intersect(surfaceA, surfaceB, tolerance, isoDivisions);
			var curves = new List<NurbsCurve>();

			if (pts.Count == 0) return curves;

			// Produce single 3D point for each pair (average of PointA and PointB)
			var points = pts.Select(p => new Vector3Double((p.PointA.X + p.PointB.X) * 0.5,
														  (p.PointA.Y + p.PointB.Y) * 0.5,
														  (p.PointA.Z + p.PointB.Z) * 0.5)).ToList();

			// Cluster points by proximity into chains
			double clusterThreshold = Math.Max(1e-3, tolerance * 1000.0);
			var clusters = ClusterPoints(points.ToArray(), clusterThreshold);

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
					for (int j = 0; j < n; j++)
					{
						if (visited[j]) continue;
						if ((points[j] - points[idx]).magnitude < threshold)
						{
							visited[j] = true;
							queue.Enqueue(j);
						}
					}
				}
				clusters.Add(cluster);
			}

			return clusters;
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
