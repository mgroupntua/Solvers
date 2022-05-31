using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;

namespace MGroup.Solvers.DofOrdering.Reordering
{
    /// <summary>
    /// Reorders the unconstrained freedom degrees by using multiple other reordering strategies. For now, only consecutive 
    /// application of each reordering strategy is possible.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public class CompositeReordering : IDofReorderingStrategy
    {
        private readonly IReadOnlyList<IDofReorderingStrategy> reorderingStrategies;

        public CompositeReordering(params IDofReorderingStrategy[] reorderingStrategies)
        {
            this.reorderingStrategies = reorderingStrategies;
        }

        public void ReorderDofs(ISubdomain subdomain, ISubdomainFreeDofOrdering originalOrdering)
        {
            foreach (var reordering in reorderingStrategies) reordering.ReorderDofs(subdomain, originalOrdering);
        }
    }
}
