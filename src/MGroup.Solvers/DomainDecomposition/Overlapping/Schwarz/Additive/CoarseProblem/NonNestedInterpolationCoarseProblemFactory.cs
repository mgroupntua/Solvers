using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.CoarseProblem
{
    public class NonNestedInterpolationCoarseProblemFactory : ICoarseProblemFactory
    {
        public void GenerateMatrices(IMatrixView matrix, IModelOverlappingDecomposition modelOverlappingDecomposition)
        {
            throw new NotImplementedException();
        }

        public Matrix RetrievePreconditionerContribution()
        {
            throw new NotImplementedException();
        }
    }
}
