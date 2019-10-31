using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DofOrdering
{
	public class SubdomainConstrainedDofOrderingGeneral : SubdomainConstrainedDofOrderingBase
    {
        public SubdomainConstrainedDofOrderingGeneral(int numConstrainedDofs, DofTable subdomainConstrainedDofs):
            base(numConstrainedDofs, subdomainConstrainedDofs) { }

        public override (int[] elementDofIndices, int[] subdomainDofIndices) 
            MapConstrainedDofsElementToSubdomain(IElement element)
        {
            (int numAllElementDofs, int[] elementDofIndices, int[] subdomainDofIndices) = 
                base.ProcessConstrainedDofsOfElement(element);
            return (elementDofIndices, subdomainDofIndices);
        }
    }
}
