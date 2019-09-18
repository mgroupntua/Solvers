using MGroup.MSolve.Discretization.FreedomDegrees;
using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.Ordering.Reordering
{
    /// <summary>
    /// Reorders the unconstrained freedom degrees of a subdomain.
    /// Author: Serafeim Bakalakos
    /// </summary>
    public interface IDofReorderingStrategy
    {
        /// <summary>
        /// Reorders the unconstrained freedom degrees of a subdomain, given an existing ordering.
        /// </summary>
        /// <param name="subdomain">The subdomain whose freedom degrees will be reordered.</param>
        /// <param name="originalOrdering">The current ordering of the subdomain's freedom degrees. It will be modified.</param>
        void ReorderDofs(ISubdomain subdomain, ISubdomainFreeDofOrdering originalOrdering);
    }
}
