using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DomainDecomposition.Dual.FetiDP.InterfaceProblem;

namespace MGroup.Solvers.DomainDecomposition.Dual.FetiDP.Matrices
{
    public interface IFetiDPSubdomainMatrixManagerFactory
    {
        //TODO: This is in a different namespace
        IFetiDPCoarseProblemSolver CreateCoarseProblemSolver(IReadOnlyList<ISubdomain> subdomains); 

        IFetiDPSubdomainMatrixManager CreateMatricesManager(ISubdomain subdomain);
    }
}
