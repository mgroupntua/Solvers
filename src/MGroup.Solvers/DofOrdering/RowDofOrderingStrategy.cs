using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Commons;

namespace MGroup.Solvers.DofOrdering
{
	public class RowDofOrderingStrategy : IAsymmetricDofOrderingStrategy
	{
		public (int numGlobalFreeDofs, DofTable globalFreeDofs) OrderGlobalDofs(IAsymmetricModel model)
			=> OrderFreeDofsOfElementSet(model.Elements, model.Constraints);

        private (int numGlobalFreeDofs, DofTable globalFreeDofs) OrderFreeDofsOfElementSet(IReadOnlyList<IElement> elements, Table<INode, IDofType, double> constraints)
        {
            var freeDofs = new DofTable();
            int dofCounter = 0;
            foreach (var element in elements)
            {
                var collocationElement = (ICollocationElement) element;
                var collocationPoint = ((ICollocationElement)element).CollocationPoint;
                var dofsPerNode = collocationElement.GetDOFTypesForDOFEnumeration(element);
                bool isNodeConstrained = collocationPoint.Constraints?.Count != 0;
                foreach (IDofType dof in dofsPerNode)
                {
                    bool isDofConstrained = isNodeConstrained && collocationPoint.Constraints!=null&& collocationPoint.Constraints.Any(c=>c.DOF==dof);
                    if (!isDofConstrained)
                        freeDofs[collocationPoint, dof] = dofCounter++;
                }
            }
            return (dofCounter, freeDofs);
        }


		public (int numSubdomainFreeDofs, DofTable subdomainFreeDofs) OrderSubdomainDofs(IAsymmetricSubdomain subdomain)
			=> OrderFreeDofsOfElementSet(((ISubdomain)subdomain).Elements, ((ISubdomain)subdomain).Constraints);
	}
}
