using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Commons;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Interfaces;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.CoarseProblem;
using MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.Interfaces;
using MGroup.Solvers.Ordering;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive
{
    public class OverlappingAdditiveSchwarzPreconditioner : IPreconditioner
    {
        private readonly IMatrix _preconditioner;

        public OverlappingAdditiveSchwarzPreconditioner(IMatrixView matrix, ICoarseProblemFactory coarseProblemFactory,
            LocalProblemsFactory localProblemsFactory, IModelOverlappingDecomposition modelOverlappingDecomposition,
            IAsymmetricModel model)
        {
            modelOverlappingDecomposition.DecomposeMatrix();
            coarseProblemFactory.GenerateMatrices(matrix, modelOverlappingDecomposition);
            localProblemsFactory.GenerateMatrices(matrix, modelOverlappingDecomposition);
            var coarseProblemContribution = coarseProblemFactory.RetrievePreconditionerContribution();
            var localProblemsContribution = localProblemsFactory.RetrievePreconditionerContribution();

            var freeSubdomainDofs= model.GlobalRowDofOrdering.MapFreeDofsSubdomainToGlobal(model.Subdomains[0]);
            _preconditioner = coarseProblemContribution.Add(localProblemsContribution).GetSubmatrix(freeSubdomainDofs, freeSubdomainDofs);
            Order = _preconditioner.NumRows;
        }

        /// <summary>
        /// The number of rows/columns of the preconditioner and the original matrix
        /// </summary>
        public int Order { get; }

        public void SolveLinearSystem(IVectorView rhsVector, IVector lhsVector)
        {
            Preconditions.CheckSystemSolutionDimensions(Order, rhsVector.Length);
            lhsVector = _preconditioner.Multiply(rhsVector);
        }
    }

    public class Factory : IPreconditionerFactory
    {
        private readonly IAsymmetricModel _model;
        private readonly IModelOverlappingDecomposition _modelOverlappingDecomposition;
        private readonly List<IWeightedPoint> _patchControlPoints;

        public Factory(IModelOverlappingDecomposition modelOverlappingDecomposition, 
                       IAsymmetricModel model)
        {
            _model = model;
            _modelOverlappingDecomposition = modelOverlappingDecomposition;
        }

        public ICoarseProblemFactory CoarseProblemFactory { get; set; } = new NestedInterpolationCoarseProblemFactory();

        public LocalProblemsFactory LocalProblemsFactory { get; set; } = new LocalProblemsFactory();

        public IPreconditioner CreatePreconditionerFor(IMatrixView matrix) =>
            new OverlappingAdditiveSchwarzPreconditioner(matrix, CoarseProblemFactory, LocalProblemsFactory,
                _modelOverlappingDecomposition, _model);
    }
}
