using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Operators;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DomainDecomposition.Dual.LagrangeMultipliers;

//TODO: This should be an enum class. There are only 2 possible cases.
//TODO: This should work for both FETI-1 and FETI-DP
namespace MGroup.Solvers.DomainDecomposition.Dual.StiffnessDistribution 
{
    public interface IStiffnessDistribution
    {
        double[] CalcBoundaryDofCoefficients(ISubdomain subdomain);

        Dictionary<int, double> CalcBoundaryDofCoefficients(INode node, IDofType dofType);

        Dictionary<int, IMappingMatrix> CalcBoundaryPreconditioningSignedBooleanMatrices(
            ILagrangeMultipliersEnumerator lagrangeEnumerator, 
            Dictionary<int, SignedBooleanMatrixColMajor> boundarySignedBooleanMatrices);

        void Update(Dictionary<int, IMatrixView> stiffnessesFreeFree);
    }
}
