using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.DofOrdering;

namespace MGroup.Solvers.DofOrdering
{
    public interface IAsymmetricDofOrderer
    {
        ISubdomainConstrainedDofOrdering OrderConstrainedDofs(ISubdomain subdomain);
        IGlobalFreeDofOrdering OrderFreeDofs(IAsymmetricModel model);
    }
}
