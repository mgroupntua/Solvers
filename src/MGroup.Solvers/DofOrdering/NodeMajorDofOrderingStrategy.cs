using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;
using System;
using System.Runtime.InteropServices;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution.AlgebraicModel;

//TODO: benchmark this against simple ordering + node major reordering
namespace MGroup.Solvers.DofOrdering
{
	/// <summary>
	/// Free dofs are assigned global / subdomain indices in a node major fashion: The dofs of the first node are 
	/// numbered, then the dofs of the second node, etc. Constrained dofs are ignored.
	/// </summary>
	public class NodeMajorDofOrderingStrategy : IFreeDofOrderingStrategy
	{
		public (int numSubdomainFreeDofs, IntDofTable subdomainFreeDofs) OrderSubdomainDofs(ISubdomain subdomain, IAlgebraicModelInterpreter boundaryConditionsInterpreter)
		{
			var nodalDOFTypesDictionary = new Dictionary<int, List<IDofType>>(); //TODO: use Set instead of List
			foreach (IElementType element in subdomain.EnumerateElements())
			{
				for (int i = 0; i < element.Nodes.Count; i++)
				{
					if (!nodalDOFTypesDictionary.ContainsKey(element.Nodes[i].ID))
						nodalDOFTypesDictionary.Add(element.Nodes[i].ID, new List<IDofType>());
					nodalDOFTypesDictionary[element.Nodes[i].ID].AddRange(element.DofEnumerator.GetDofTypesForDofEnumeration(element)[i]);
				}
			}

			int dofIdx = 0;
			var constrainedDofs = boundaryConditionsInterpreter.GetDirichletBoundaryConditionsWithNumbering(subdomain.ID);
			var freeDofs = new IntDofTable();
			foreach (INode node in subdomain.EnumerateNodes())
			{
				foreach (IDofType dofType in nodalDOFTypesDictionary[node.ID].Distinct())
				{
					if (constrainedDofs == null || constrainedDofs.ContainsKey((node.ID, dofType)) == false)
					{
						freeDofs[node.ID, boundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dofType)] = dofIdx++;
					}
				}
			}

			return (dofIdx, freeDofs);
		}

		//public (int numSubdomainFreeDofs, IntDofTable subdomainFreeDofs) OrderSubdomainDofs(ISubdomain subdomain,
		//	ActiveDofs allDofs)
		//	=> OrderFreeDofsOfElementSet(subdomain.EnumerateElements(), subdomain.EnumerateNodes(), allDofs);

		// Copied from the methods used by Subdomain and Model previously.
		//private static (int numFreeDofs, IntDofTable freeDofs) OrderFreeDofsOfElementSet(IEnumerable<IElementType> elements,
		//	IEnumerable<INode> sortedNodes, ActiveDofs allDofs)
		//{
		//	int totalDOFs = 0;
		//	Dictionary<int, List<IDofType>> nodalDOFTypesDictionary = new Dictionary<int, List<IDofType>>(); //TODO: use Set instead of List
		//	foreach (IElementType element in elements)
		//	{
		//		for (int i = 0; i < element.Nodes.Count; i++)
		//		{
		//			if (!nodalDOFTypesDictionary.ContainsKey(element.Nodes[i].ID))
		//				nodalDOFTypesDictionary.Add(element.Nodes[i].ID, new List<IDofType>());
		//			nodalDOFTypesDictionary[element.Nodes[i].ID].AddRange(element.DofEnumerator.GetDofTypesForDofEnumeration(element)[i]);
		//		}
		//	}

		//	var freeDofs = new IntDofTable();
		//	foreach (INode node in sortedNodes)
		//	{
		//		//List<DOFType> dofTypes = new List<DOFType>();
		//		//foreach (Element element in node.ElementsDictionary.Values)
		//		//{
		//		//    if (elementsDictionary.ContainsKey(element.ID))
		//		//    {
		//		//        foreach (DOFType dof in element.ElementType.DOFTypes)
		//		//            dofTypes.Add(dof);
		//		//    }
		//		//}

		//		Dictionary<IDofType, int> dofsDictionary = new Dictionary<IDofType, int>();
		//		//foreach (DOFType dofType in dofTypes.Distinct<DOFType>())
		//		foreach (IDofType dofType in nodalDOFTypesDictionary[node.ID].Distinct())
		//		{
		//			int dofIdx = 0;
		//			#region removeMaria
		//			//foreach (DOFType constraint in node.Constraints)
		//			//{
		//			//    if (constraint == dofType)
		//			//    {
		//			//        dofID = -1;
		//			//        break;
		//			//    }
		//			//}
		//			#endregion

		//			foreach (var constraint in node.Constraints) //TODO: access the constraints from the subdomain
		//			{
		//				if (constraint.DOF == dofType)
		//				{
		//					dofIdx = -1;
		//					break;
		//				}
		//			}

		//			//var embeddedNode = embeddedNodes.Where(x => x.Node == node).FirstOrDefault();
		//			////if (node.EmbeddedInElement != null && node.EmbeddedInElement.ElementType.GetDOFTypes(null)
		//			////    .SelectMany(d => d).Count(d => d == dofType) > 0)
		//			////    dofID = -1;
		//			//if (embeddedNode != null && embeddedNode.EmbeddedInElement.ElementType.DOFEnumerator.GetDOFTypes(null)
		//			//    .SelectMany(d => d).Count(d => d == dofType) > 0)
		//			//    dofID = -1;

		//			if (dofIdx == 0)
		//			{
		//				dofIdx = totalDOFs;
		//				freeDofs[node.ID, allDofs.GetIdOfDof(dofType)] = dofIdx;
		//				totalDOFs++;
		//			}
		//		}
		//	}

		//	return (totalDOFs, freeDofs);
		//}
	}
}
