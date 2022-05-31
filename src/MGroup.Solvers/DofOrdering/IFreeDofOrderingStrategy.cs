using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.Solvers.DofOrdering
{
    /// <summary>
    /// Determines how the unconstrained freedom degrees of the physical model will be ordered.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public interface IFreeDofOrderingStrategy
    {
		/// <summary>
		/// Orders the unconstrained freedom degrees of one of the model's subdomains.
		/// </summary>
		/// <param name="subdomain">A subdomain of the whole model.</param>
		/// <param name="boundaryConditionsInterpreter">A <see cref="IAlgebraicModelInterpreter"/> for defining the constrained freedom degrees of the subdomain/>.</param>
		(int numSubdomainFreeDofs, IntDofTable subdomainFreeDofs) OrderSubdomainDofs(ISubdomain subdomain, IAlgebraicModelInterpreter boundaryConditionsInterpreter);
    }
}
