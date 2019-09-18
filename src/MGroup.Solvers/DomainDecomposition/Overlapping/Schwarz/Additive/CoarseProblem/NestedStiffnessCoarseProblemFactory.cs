using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.CoarseProblem
{
    public class NestedStiffnessCoarseProblemFactory : ICoarseProblemFactory
    {
        public void GenerateMatrices(IMatrixView matrix, IModelOverlappingDecomposition modelOverlappingDecomposition)
        {
            throw new System.NotImplementedException();
        }

        public Matrix RetrievePreconditionerContribution()
        {
            throw new System.NotImplementedException();
        }
    }
}
