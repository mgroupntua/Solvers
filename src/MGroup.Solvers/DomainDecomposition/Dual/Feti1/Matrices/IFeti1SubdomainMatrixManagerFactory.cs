using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DomainDecomposition.Dual.Feti1.Matrices
{
    public interface IFeti1SubdomainMatrixManagerFactory
    {
        IFeti1SubdomainMatrixManager CreateMatricesManager(ISubdomain subdomain);
    }
}
