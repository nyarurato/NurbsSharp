using System;
using System.Collections.Generic;

namespace UnitTests.TestInfrastructure
{
    internal readonly record struct ParameterDomain(double Start, double End)
    {
        internal double Span => End - Start;

        internal double At(double normalizedParameter)
        {
            if (normalizedParameter < 0.0 || normalizedParameter > 1.0)
                throw new ArgumentOutOfRangeException(nameof(normalizedParameter));

            return Start + normalizedParameter * Span;
        }
    }

    internal static class DomainSamples
    {
        internal static IReadOnlyList<double> EndpointsAndInterior(ParameterDomain domain)
        {
            return
            [
                domain.Start,
                domain.At(0.25),
                domain.At(0.5),
                domain.At(0.75),
                domain.End,
            ];
        }

        internal static IReadOnlyList<double> OneSidedEndpoints(ParameterDomain domain)
        {
            return
            [
                domain.Start,
                Math.BitIncrement(domain.Start),
                Math.BitDecrement(domain.End),
                domain.End,
            ];
        }

        internal static IReadOnlyList<double> AroundInteriorKnot(double knot)
        {
            return [Math.BitDecrement(knot), knot, Math.BitIncrement(knot)];
        }
    }
}
