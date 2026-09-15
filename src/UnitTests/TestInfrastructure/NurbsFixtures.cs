using System;
using NurbsSharp.Core;
using NurbsSharp.Geometry;

namespace UnitTests.TestInfrastructure
{
    internal static class NurbsFixtures
    {
        internal static NurbsCurve ArbitraryDomainLine()
        {
            return new NurbsCurve(
                1,
                new KnotVector([2.0, 2.0, 5.0, 5.0], 1),
                [new ControlPoint(1.0, 2.0, 3.0), new ControlPoint(5.0, 6.0, 7.0)]);
        }

        internal static NurbsCurve QuadraticBezier()
        {
            return new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(0.0, 0.0, 0.0),
                    new ControlPoint(1.0, 2.0, 0.0),
                    new ControlPoint(3.0, 0.0, 0.0),
                ]);
        }

        internal static NurbsCurve QuarterCircle()
        {
            double middleWeight = Math.Sqrt(2.0) / 2.0;
            return new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(1.0, 0.0, 0.0, 1.0),
                    new ControlPoint(1.0, 1.0, 0.0, middleWeight),
                    new ControlPoint(0.0, 1.0, 0.0, 1.0),
                ]);
        }

        internal static NurbsCurve ConstantRationalCurve()
        {
            return new NurbsCurve(
                2,
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    new ControlPoint(7.0, -2.0, 0.5, 1.0),
                    new ControlPoint(7.0, -2.0, 0.5, 2.0),
                    new ControlPoint(7.0, -2.0, 0.5, 0.5),
                ]);
        }

        internal static NurbsSurface ArbitraryDomainRectangle()
        {
            return new NurbsSurface(
                1,
                1,
                new KnotVector([2.0, 2.0, 5.0, 5.0], 1),
                new KnotVector([10.0, 10.0, 14.0, 14.0], 1),
                [
                    [new ControlPoint(0.0, 0.0, 0.0), new ControlPoint(0.0, 3.0, 0.0)],
                    [new ControlPoint(4.0, 0.0, 0.0), new ControlPoint(4.0, 3.0, 0.0)],
                ]);
        }

        internal static NurbsSurface QuarterCylinder()
        {
            double middleWeight = Math.Sqrt(2.0) / 2.0;
            return new NurbsSurface(
                1,
                2,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new KnotVector([0.0, 0.0, 0.0, 1.0, 1.0, 1.0], 2),
                [
                    [
                        new ControlPoint(1.0, 0.0, 0.0, 1.0),
                        new ControlPoint(1.0, 1.0, 0.0, middleWeight),
                        new ControlPoint(0.0, 1.0, 0.0, 1.0),
                    ],
                    [
                        new ControlPoint(1.0, 0.0, 2.0, 1.0),
                        new ControlPoint(1.0, 1.0, 2.0, middleWeight),
                        new ControlPoint(0.0, 1.0, 2.0, 1.0),
                    ],
                ]);
        }

        internal static NurbsSurface ConstantSurface()
        {
            return new NurbsSurface(
                1,
                1,
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                new KnotVector([0.0, 0.0, 1.0, 1.0], 1),
                [
                    [new ControlPoint(7.0, -2.0, 0.5, 1.0), new ControlPoint(7.0, -2.0, 0.5, 2.0)],
                    [new ControlPoint(7.0, -2.0, 0.5, 3.0), new ControlPoint(7.0, -2.0, 0.5, 0.5)],
                ]);
        }

        internal static NurbsVolume IdentityTrilinearVolume(
            double uStart = 0.0,
            double uEnd = 1.0,
            double vStart = 0.0,
            double vEnd = 1.0,
            double wStart = 0.0,
            double wEnd = 1.0)
        {
            ControlPoint[][][] controlPoints = new ControlPoint[2][][];
            for (int i = 0; i < 2; i++)
            {
                controlPoints[i] = new ControlPoint[2][];
                for (int j = 0; j < 2; j++)
                {
                    controlPoints[i][j] = new ControlPoint[2];
                    for (int k = 0; k < 2; k++)
                        controlPoints[i][j][k] = new ControlPoint(i, j, k);
                }
            }

            return new NurbsVolume(
                1,
                1,
                1,
                new KnotVector([uStart, uStart, uEnd, uEnd], 1),
                new KnotVector([vStart, vStart, vEnd, vEnd], 1),
                new KnotVector([wStart, wStart, wEnd, wEnd], 1),
                controlPoints);
        }

        internal static NurbsVolume RationalTrilinearVolume()
        {
            NurbsVolume volume = IdentityTrilinearVolume();
            volume.ControlPoints[0][0][0].Weight = 2.0;
            return volume;
        }
    }
}
