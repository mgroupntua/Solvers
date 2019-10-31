using MGroup.MSolve.Discretization;

//TODO: This could be done simultaneously with ordering the free dofs, to improve performance.
namespace MGroup.Solvers.DofOrdering
{
    /// <summary>
    /// Orders the constrained dofs of a subdomain, independendtly from the free ones.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    internal class ConstrainedDofOrderingStrategy
    {
        /// <summary>
        /// Orders the constrained freedom degrees of one of the model's subdomains.
        /// </summary>
        /// <param name="subdomain">A subdomain of the whole model.</param>
        internal (int numSubdomainConstrainedDofs, DofTable subdomainConstrainedDofs) OrderSubdomainDofs(ISubdomain subdomain)
        {
            var constrainedDofs = new DofTable();
            int dofCounter = 0;
            foreach (INode node in subdomain.Nodes)
            {
                foreach (Constraint constraint in node.Constraints) constrainedDofs[node, constraint.DOF] = dofCounter++;
            }
            return (dofCounter, constrainedDofs);
        }
    }
}
