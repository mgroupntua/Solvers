using MGroup.MSolve.Discretization.FreedomDegrees;
using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.Ordering.Reordering
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
