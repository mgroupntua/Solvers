using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive;

using Xunit;

namespace MGroup.Solvers.Tests
{
	public class OverlappingSchwarzTests
    {
        [Fact]
        public void ParametricAxisDecompositionTest()
        {
            var knotValueVector = Vector.CreateFromArray(new double[]
                {0, 0, 0, 0, 1 / 6.0, 1 / 3.0, 1 / 2.0, 2 / 3.0, 5 / 6.0, 1, 1, 1, 1});

            var d = new ParametricAxisDecomposition(knotValueVector, 3, 2);
            d.DecomposeAxis();
        }
    }
}
