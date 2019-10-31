using System.Collections.Generic;
using MGroup.Solvers.DomainDecomposition.Dual.StiffnessDistribution;
using MGroup.Solvers.DomainDecomposition.Dual.LagrangeMultipliers;
using MGroup.Solvers.DomainDecomposition.Dual.Pcg;
using MGroup.MSolve.Discretization;

//TODO: Also add ReorderInternalDofsForMultiplication and ReorderBoundaryDofsForMultiplication
namespace MGroup.Solvers.DomainDecomposition.Dual.Preconditioning
{
    public interface IFetiPreconditionerFactory
    {
        bool ReorderInternalDofsForFactorization { get; }

        IFetiPreconditioner CreatePreconditioner(IModel model, IStiffnessDistribution stiffnessDistribution,
            IDofSeparator dofSeparator, ILagrangeMultipliersEnumerator lagrangeEnumerator, 
            Dictionary<int, IFetiSubdomainMatrixManager> matrixManagers);
    }
}
