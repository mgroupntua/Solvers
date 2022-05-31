using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.Solvers.DofOrdering
{
	/// <summary>
	/// Free dofs are assigned global / subdomain indices in a node major fashion: The dofs of the first node are numbered, then 
	/// the dofs of the second node, etc. Note that the dofs of each node are assumed to be the same and supplied by the client. 
	/// Based on that assumption, this class is much faster than its alternatives. Constrained dofs are ignored.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	public class UniformDofOrderingStrategy : IFreeDofOrderingStrategy
	{
		private readonly IReadOnlyList<IDofType> dofsPerNode;

		public UniformDofOrderingStrategy(IReadOnlyList<IDofType> dofsPerNode)
		{
			this.dofsPerNode = dofsPerNode;
		}

		public (int numSubdomainFreeDofs, IntDofTable subdomainFreeDofs) OrderSubdomainDofs(ISubdomain subdomain, IAlgebraicModelInterpreter boundaryConditionsInterpreter)
		{
			var constrainedDofs = boundaryConditionsInterpreter.GetDirichletBoundaryConditionsWithNumbering(subdomain.ID);
			var freeDofs = new IntDofTable();
			int dofIdx = 0;
			foreach (INode node in subdomain.EnumerateNodes())
			{
				foreach (IDofType dof in dofsPerNode)
				{
					if (!constrainedDofs.ContainsKey((node.ID, dof)))
					{
						freeDofs[node.ID, boundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dof)] = dofIdx++;
					}
				}
			}
			return (dofIdx, freeDofs);
		}

		//public (int numSubdomainFreeDofs, IntDofTable subdomainFreeDofs) OrderSubdomainDofs(
		//	ISubdomain subdomain, ActiveDofs allDofs)
		//	=> OrderFreeDofsOfNodeSet(subdomain.EnumerateNodes(), allDofs);

		//private (int numFreeDofs, IntDofTable freeDofs) OrderFreeDofsOfNodeSet(IEnumerable<INode> sortedNodes, ActiveDofs allDofs)
		//{
		//	var freeDofs = new IntDofTable();
		//	int dofIdx = 0;
		//	foreach (INode node in sortedNodes)
		//	{
		//		var constrainedDofs = new HashSet<IDofType>();
		//		foreach (Constraint constraint in node.Constraints)
		//		{
		//			constrainedDofs.Add(constraint.DOF);
		//		}

		//		foreach (IDofType dof in dofsPerNode)
		//		{
		//			if (!constrainedDofs.Contains(dof))
		//			{
		//				freeDofs[node.ID, allDofs.GetIdOfDof(dof)] = dofIdx++;
		//			}
		//		}
		//	}
		//	return (dofIdx, freeDofs);
		//}
	}
}
