using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Discretization.Providers;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.Assemblers
{
	public class SubdomainVectorAssembler
	{
		private readonly ActiveDofs allDofs;

		public SubdomainVectorAssembler(ActiveDofs allDofs)
		{
			this.allDofs = allDofs;
		}

		public void AddToSubdomainVector(IEnumerable<IElementType> elements, Vector subdomainVector,
			IElementVectorProvider vectorProvider, ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (IElementType element in elements)
			{
				(int[] elementDofs, int[] subdomainDofs) = dofOrdering.MapFreeDofsElementToSubdomain(element);
				var elementVector = Vector.CreateFromArray(vectorProvider.CalcVector(element));
				subdomainVector.AddIntoThisNonContiguouslyFrom(subdomainDofs, elementVector, elementDofs);
			}
		}
		
		//public void AddToSubdomainVector(IEnumerable<IElementBoundaryCondition> loads, Vector subdomainVector,
		//	ISubdomainFreeDofOrdering dofOrdering)
		//{
		//	foreach (IElementBoundaryCondition load in loads)
		//	{
		//		(int[] elementDofs, int[] subdomainDofs) = dofOrdering.MapFreeDofsElementToSubdomain(load.Element);
		//		var elementVector = Vector.CreateFromArray(load.EnumerateNodalLoads());
		//		subdomainVector.AddIntoThisNonContiguouslyFrom(subdomainDofs, elementVector, elementDofs);
		//	}
		//}

		public void AddToSubdomainVector(IEnumerable<INodalModelQuantity<IDofType>> loads, Vector subdomainVector,
			ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (var load in loads)
			{
				int dofIdx = dofOrdering.FreeDofs[load.Node.ID, allDofs.GetIdOfDof(load.DOF)];
				subdomainVector[dofIdx] += load.Amount;
			}
		}

		public void AddToSubdomainVector(IEnumerable<IDomainModelQuantity<IDofType>> loads, Vector subdomainVector,
			ISubdomainFreeDofOrdering dofOrdering)
		{
			foreach (int node in dofOrdering.FreeDofs.GetRows())
			{
				foreach (var dofIdxPair in dofOrdering.FreeDofs.GetDataOfRow(node))
				{
					int dof = dofIdxPair.Key;
					int idx = dofIdxPair.Value;

					foreach (var load in loads)
					{
						if (allDofs.GetIdOfDof(load.DOF) == dof)
						{
							subdomainVector[idx] = load.Amount;
						}
					}
				}
			}
		}
	}
}
