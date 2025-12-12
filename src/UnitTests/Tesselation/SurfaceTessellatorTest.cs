using NurbsSharp.Core;
using NurbsSharp.Tesselation;
using NurbsSharp.Geometry;
using NurbsSharp.Generation;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NurbsSharp.IO;
using System.IO;
using NUnit.Framework.Internal;

namespace UnitTests.Tesselation
{
    [TestFixture]
    internal class SurfaceTessellatorTest
    {
        [Test]
        public void TessellateAdaptive_QuadFace_PlanarProducesTwoTriangles()
        {
            var p00 = new Vector3Double(0, 0, 0);
            var p01 = new Vector3Double(10, 0, 0);
            var p10 = new Vector3Double(0, 10, 0);
            var p11 = new Vector3Double(10, 10, 0);
            var face = PrimitiveFactory.CreateFace(p00, p01, p10, p11);
            var mesh = SurfaceTessellator.TessellateAdaptive(face, 1e-8);
            // planar bilinear face should produce exactly two triangles with no subdivisions
            Assert.That(mesh.Indexes.Length, Is.EqualTo(6));
            Assert.That(mesh.Vertices.Length, Is.EqualTo(4));
        }

        [Test]
        public void TessellateAdaptive_Sphere_FewerTrianglesThanUniform()
        {
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            var uniform = SurfaceTessellator.Tessellate(sphere, 16, 16);
            // use a relatively coarse tolerance so the adaptive tessellator reduces triangles
            var adaptive = SurfaceTessellator.TessellateAdaptive(sphere, tolerance: 0.2, maxDepth: 4);
            Assert.That(adaptive.Indexes.Length, Is.LessThan(uniform.Indexes.Length));
            // also verify that adaptive mesh approximates sphere: sample a vertex and check distance
            // compute approximate radius at a vertex
            var v = adaptive.Vertices.FirstOrDefault();
            Assert.That(Math.Abs(Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z) - radius) < radius * 0.5);
        }

        [Test]
        public void TessellateAdaptiveVSNormal_Sphere_CompareTriangleCounts()
        {
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            var adaptive = SurfaceTessellator.TessellateAdaptive(sphere, tolerance: 0.1, maxDepth: 5);
            var normal = SurfaceTessellator.Tessellate(sphere, numPointsU: 50, numPointsV: 50);
            Assert.That(adaptive.Indexes, Has.Length.LessThan(normal.Indexes.Length));
        }

        [Test]
        public void TessellateAdaptiveVSNormal_p3surf_CompareTriangleCounts()
        {
            int degreeU = 3;
            int degreeV = 3;
            double[] knotsU = { 0, 0, 0, 0, 0.5,1, 1, 1, 1 };
            double[] knotsV = { 0, 0, 0, 0, 0.5,1, 1, 1, 1 };
            KnotVector knotVectorU = new KnotVector(knotsU, degreeU);
            KnotVector knotVectorV = new KnotVector(knotsV, degreeV);
            ControlPoint[][] controlPoints = new ControlPoint[5][]; // 5x5 control points U x V
            controlPoints[0] = new ControlPoint[] {
            // x, y, z, weight
            new ControlPoint(0.0, 0.0, 0.0, 1),  //U0 V0
            new ControlPoint(1.0, 0.0, 0.0, 1),  //U0 V1
            new ControlPoint(2.0, 0.0, 0.0, 1),  //U0 V2
            new ControlPoint(3.0, 0.0, 0.0, 1),  //U0 V3
            new ControlPoint(4.0, 0.0, 0.0, 1)   //U0 V4
            };
            controlPoints[1] = new ControlPoint[] {
            new ControlPoint(0.0, 1.0, 0.5, 1),  //U1 V0
            new ControlPoint(1.0, 1.0, -1.5, 1), //U1 V1
            new ControlPoint(2.0, 1.0, 4.0, 1),  //U1 V2
            new ControlPoint(3.0, 1.0, -3.0, 1), //U1 V3
            new ControlPoint(4.0, 1.0, 0.5, 1)   //U1 V4
            };
            controlPoints[2] = new ControlPoint[] {
            new ControlPoint(0.0, 2.0, 1.5, 1),  //U2 V0
            new ControlPoint(1.0, 2.0, 2.5, 1),  //U2 V1
            new ControlPoint(2.0, 2.0, 3.5, 0.7),//U2 V2
            new ControlPoint(3.0, 2.0, 3.0, 1),  //U2 V3
            new ControlPoint(4.0, 2.0, 0.0, 1)   //U2 V4
            };
            controlPoints[3] = new ControlPoint[] {
            new ControlPoint(0.0, 3.0, 0.5, 1),  //U3 V0
            new ControlPoint(1.5, 3.0, -1.5, 1), //U3 V1
            new ControlPoint(2.5, 3.0, 2.0 ,1),  //U3 V2
            new ControlPoint(3.5, 3.0, -1.5, 1), //U3 V3
            new ControlPoint(4.5, 3.0, -1.0, 1)  //U3 V4
            };
            controlPoints[4] = new ControlPoint[] {
            new ControlPoint(0.0, 4.0, 0.5, 1),  //U4 V0
            new ControlPoint(1.0, 4.0, 0.5, 1),  //U4 V1
            new ControlPoint(2.0, 4.0, 0.0, 1),  //U4 V2
            new ControlPoint(3.0, 4.0, 0.0, 1),  //U4 V3
            new ControlPoint(4.0, 4.0, 0.0, 1)   //U4 V4
            };

            NurbsSurface surface = new NurbsSurface(degreeU, degreeV, knotVectorU, knotVectorV, controlPoints);
            
            var adaptive = SurfaceTessellator.TessellateAdaptive(surface, tolerance: 0.1, maxDepth: 5);
            var normal = SurfaceTessellator.Tessellate(surface, numPointsU: 50, numPointsV: 50);
            Assert.That(adaptive.Indexes, Has.Length.LessThan(normal.Indexes.Length));
        }

        [Test]
        public void TessellateAdaptive_CurvatureBasedSubdivision_Verification()
        {
            // Test with two different radii spheres - smaller radius should have more triangles
            double smallRadius = 2.0;
            double largeRadius = 10.0;
            
            var smallSphere = PrimitiveFactory.CreateSphere(smallRadius);
            var largeSphere = PrimitiveFactory.CreateSphere(largeRadius);
            
            // Small radius has higher curvature (1/radius), so should subdivide more with same tolerance
            // curvature = 1/radius: small=0.5, large=0.1
            double tolerance = 0.3; // subdivide if curvature > 0.3 (radius < 3.33)
            
            var smallMesh = SurfaceTessellator.TessellateAdaptive(smallSphere, tolerance, maxDepth: 6);
            var largeMesh = SurfaceTessellator.TessellateAdaptive(largeSphere, tolerance, maxDepth: 6);
            
            Console.WriteLine($"Small sphere (r={smallRadius}, k={1.0/smallRadius:F2}): {smallMesh.Indexes.Length/3} triangles");
            Console.WriteLine($"Large sphere (r={largeRadius}, k={1.0/largeRadius:F2}): {largeMesh.Indexes.Length/3} triangles");
            
            // Small sphere should have significantly more triangles due to higher curvature
            Assert.That(smallMesh.Indexes.Length, Is.GreaterThan(largeMesh.Indexes.Length),
                "Smaller radius (higher curvature) should produce more triangles");
        }

        [Test]
        public void TessellateAdaptive_ToleranceEffect_StricterToleranceMoreTriangles()
        {
            double radius = 5.0;
            var sphere = PrimitiveFactory.CreateSphere(radius);
            
            // Test with different tolerance values
            double looseTolerance = 0.5;   // subdivide if curvature > 0.5 (radius < 2)
            double strictTolerance = 0.1;  // subdivide if curvature > 0.1 (radius < 10)
            
            var looseMesh = SurfaceTessellator.TessellateAdaptive(sphere, looseTolerance, maxDepth: 6);
            var strictMesh = SurfaceTessellator.TessellateAdaptive(sphere, strictTolerance, maxDepth: 6);
            
            Console.WriteLine($"Loose tolerance ({looseTolerance}): {looseMesh.Indexes.Length/3} triangles");
            Console.WriteLine($"Strict tolerance ({strictTolerance}): {strictMesh.Indexes.Length/3} triangles");
            
            // Stricter tolerance should produce more triangles
            Assert.That(strictMesh.Indexes.Length, Is.GreaterThan(looseMesh.Indexes.Length),
                "Stricter tolerance should produce more triangles");
        }
    
        public async Task TestSTLExport(Mesh mesh, string filename="test_output.stl")
        {
            using var exporter = new FileStream(filename, FileMode.Create, FileAccess.Write);
            await STLExporter.ExportAsync(mesh,exporter);
        }
    }
}
