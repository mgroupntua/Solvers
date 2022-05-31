using System.Collections.Generic;
using System.Linq;
using MGroup.LinearAlgebra.Reordering;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;

//TODO: This should be thread safe.
namespace MGroup.Solvers.DofOrdering
{
	public class SubdomainFreeDofOrderingCaching : ISubdomainFreeDofOrdering
    {
        private readonly ActiveDofs allDofs;
        private readonly Dictionary<IElementType, (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices)> 
            elementDofsCache = new Dictionary<IElementType, (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices)>();

        public SubdomainFreeDofOrderingCaching(int numFreeDofs, IntDofTable subdomainFreeDofs, ActiveDofs allDofs)
		{
            this.NumFreeDofs = numFreeDofs;
            this.FreeDofs = subdomainFreeDofs;
			this.allDofs = allDofs;
        }

		public IntDofTable FreeDofs { get; }

        public int NumFreeDofs { get; }

        public void AddVectorElementToSubdomain(IElementType element, double[] elementVector, IVector subdomainVector)
        {
            (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices) = GetElementData(element);
            subdomainVector.AddIntoThisNonContiguouslyFrom(
                subdomainDofIndices, Vector.CreateFromArray(elementVector), elementDofIndices);
        }

        public int CountElementDofs(IElementType element)
        {
            (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices) = GetElementData(element);
            return numAllDofs;
        }

        public double[] ExtractVectorElementFromSubdomain(IElementType element, IVectorView subdomainVector)
        {
            (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices) = GetElementData(element);
            var elementVector = new double[numAllDofs];
            Vector.CreateFromArray(elementVector).CopyNonContiguouslyFrom(
                elementDofIndices, subdomainVector, subdomainDofIndices);
            return elementVector;

            //double[] elementVector = new double[numAllDofs];
            //for (int i = 0; i < elementDofIndices.Length; ++i)
            //{
            //    int elementDofIdx = elementDofIndices[i];
            //    int subdomainDofIdx = subdomainDofIndices[i];
            //    elementVector[elementDofIdx] = subdomainVector[subdomainDofIdx];
            //}
            //return elementVector;
        }

        public (int[] elementDofIndices, int[] subdomainDofIndices) MapFreeDofsElementToSubdomain(IElementType element)
        {
            (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices) = GetElementData(element);
            return (elementDofIndices, subdomainDofIndices);
        }

        public void Reorder(IReorderingAlgorithm reorderingAlgorithm, ISubdomain subdomain)
        {
            elementDofsCache.Clear();
            var pattern = SparsityPatternSymmetric.CreateEmpty(NumFreeDofs);
            foreach (var element in subdomain.EnumerateElements())
            {
                // Do not cache anything at this point
                (int numAllElementDofs, int[] elementDofIndices, int[] subdomainDofIndices) = ProcessElement(element);

                //TODO: ISubdomainFreeDofOrdering could perhaps return whether the subdomainDofIndices are sorted or not.
                pattern.ConnectIndices(subdomainDofIndices, false);
            }
            (int[] permutation, bool oldToNew) = reorderingAlgorithm.FindPermutation(pattern);
            FreeDofs.Reorder(permutation, oldToNew);
        }

		public void ReorderNodeMajor(IEnumerable<INode> sortedNodes)
		{
			elementDofsCache.Clear();
			FreeDofs.ReorderNodeMajor(sortedNodes.Select(n => n.ID).ToList());
        }

        private (int numAllDofs, int[] elementDofIndices, int[] subdomainDofIndices) GetElementData(IElementType element)
        {
            bool isStored = elementDofsCache.TryGetValue(element, out (int, int[], int[]) elementData);
            if (isStored) return elementData;
            else
            {
                elementData = ProcessElement(element);
                elementDofsCache.Add(element, elementData);
                return elementData;
            }
        }

        private (int numAllElementDofs, int[] elementDofIndices, int[] subdomainDofIndices) ProcessElement(IElementType element)
        {
            IReadOnlyList<INode> elementNodes = element.DofEnumerator.GetNodesForMatrixAssembly(element);
            IReadOnlyList<IReadOnlyList<IDofType>> elementDofs = element.DofEnumerator.GetDofTypesForMatrixAssembly(element);

            // Count the dof superset (free and constrained) to allocate enough memory and avoid resizing
            int allElementDofs = 0;
            for (int i = 0; i < elementNodes.Count; ++i) allElementDofs += elementDofs[i].Count;
            var elementDofIndices = new List<int>(allElementDofs);
            var subdomainDofIndices = new List<int>(allElementDofs);

            int elementDofIdx = 0;
            for (int nodeIdx = 0; nodeIdx < elementNodes.Count; ++nodeIdx)
            {
                for (int dofIdx = 0; dofIdx < elementDofs[nodeIdx].Count; ++dofIdx)
                {
					int dofID = allDofs.GetIdOfDof(elementDofs[nodeIdx][dofIdx]);
					bool isFree = FreeDofs.TryGetValue(elementNodes[nodeIdx].ID, dofID, out int subdomainDofIdx);
                    if (isFree)
                    {
                        elementDofIndices.Add(elementDofIdx);
                        subdomainDofIndices.Add(subdomainDofIdx);
                    }
                    ++elementDofIdx; // This must be incremented for constrained dofs as well
                }
            }
            return (allElementDofs, elementDofIndices.ToArray(), subdomainDofIndices.ToArray());
        }
    }
}
