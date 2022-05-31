using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.Solvers.DofOrdering
{
	/// <summary>
	/// Orders the unconstrained freedom degrees (dofs) of the physical model's subdomains, by assigning an index to each unique 
	/// (node, dof) pair. These indices are used in vectors and matrices that contain quantities for a whole subdomain to locate 
	/// the contribution of each dof.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	public interface IDofOrderer
	{
		/// <summary>
		/// Finds an ordering for the unconstrained freedom degrees of one of the physical model's subdomains.
		/// </summary>
		/// <param name="subdomain">A subdomain consisting of a subset of the nodes and elements of the total model.</param>
		/// <param name="boundaryConditionsInterpreter"></param>
		ISubdomainFreeDofOrdering OrderFreeDofs(ISubdomain subdomain, IAlgebraicModelInterpreter boundaryConditionsInterpreter);
	}
}
