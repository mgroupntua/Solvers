using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces
{
    public interface IAxisOverlappingDecomposition
    {
        void DecomposeAxis();
        double[] GetAxisCoarseKnotVector();
        double[] GetAxisCoarsePoints();
        int[] GetAxisIndicesOfSubdomain(int indexAxisSubdomain);
        int NumberOfAxisSubdomains { get; }
        int Degree { get; }
        IVector KnotValueVector { get; }
    }
}