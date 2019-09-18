using System;
using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.Dual.Feti1.Matrices
{
    public interface IFeti1SubdomainMatrixManagerFactory
    {
        IFeti1SubdomainMatrixManager CreateMatricesManager(ISubdomain subdomain);
    }
}
