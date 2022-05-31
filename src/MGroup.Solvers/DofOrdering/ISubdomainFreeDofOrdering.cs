using System.Collections.Generic;
using MGroup.LinearAlgebra.Reordering;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;

//TODO: Ideally all the relevant methods should return Vector (or at least Vector for element level) 
//TODO: The map element to subdomain should be a BiList, so that it can be read in both directions and passed to native code 
//      (e.g. CUDA). It should also be cached for each element, unless there isn't enough memory or if the dof ordering is 
//      updated frequently. It can also be used for mapping vectors (e.g. ExtractVectorElementFromSubdomain), not only the
//      stiffness matrix.
namespace MGroup.Solvers.DofOrdering
{
    public interface ISubdomainFreeDofOrdering
    {
		IntDofTable FreeDofs { get; }

        int NumFreeDofs { get; }

        //TODO: What should it contain for constrained dofs?
        void AddVectorElementToSubdomain(IElementType element, double[] elementVector, IVector subdomainVector);

        int CountElementDofs(IElementType element);

        //TODO: What should it contain for constrained dofs?
        //TODO: Should the element vector be passed in and modified instead. So far in all usecases the vector was created by 
        //      the client using CountElementDofs() immediately before passing it to this method.
        double[] ExtractVectorElementFromSubdomain(IElementType element, IVectorView subdomainVector); 

        (int[] elementDofIndices, int[] subdomainDofIndices) MapFreeDofsElementToSubdomain(IElementType element);

        //TODO: perhaps the subdomain should be passed in the constructor.
        void Reorder(IReorderingAlgorithm reorderingAlgorithm, ISubdomain subdomain);

		void ReorderNodeMajor(IEnumerable<INode> sortedNodes);
    }
}
