using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;

namespace MGroup.Solvers.DofOrdering.Reordering
{
    /// <summary>
    /// Does not apply any reordering. No object is mutated due to this class.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public class NullReordering : IDofReorderingStrategy
    {
        public void ReorderDofs(ISubdomain subdomain, ISubdomainFreeDofOrdering originalOrdering)
        {
            // Do nothing
        }
    }
}
