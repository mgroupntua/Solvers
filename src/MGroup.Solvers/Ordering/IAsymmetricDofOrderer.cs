using MGroup.MSolve.Discretization.FreedomDegrees;
using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.Ordering
{
    public interface IAsymmetricDofOrderer
    {
        ISubdomainConstrainedDofOrdering OrderConstrainedDofs(ISubdomain subdomain);
        IGlobalFreeDofOrdering OrderFreeDofs(IAsymmetricModel model);
    }
}
