using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.DofOrdering;
using MGroup.MSolve.Solution.LinearSystems;

//TODO: perhaps IFeti1SubdomainMatrixManager and IFetiDPSubdomainMatrixManager should not inherit from this one.
//TODO: perhaps I should providing access to the assembler instead of wrapping its methods.
namespace MGroup.Solvers.DomainDecomposition.Dual
{
    public interface IFetiSubdomainMatrixManager
    {
        ISingleSubdomainLinearSystem LinearSystem { get; }

        IMatrix BuildGlobalMatrix(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElement> elements,
            IElementMatrixProvider matrixProvider);

        (IMatrix Kff, IMatrixView Kfc, IMatrixView Kcf, IMatrixView Kcc) BuildGlobalSubmatrices(
            ISubdomainFreeDofOrdering freeDofOrdering, ISubdomainConstrainedDofOrdering constrainedDofOrdering,
            IEnumerable<IElement> elements, IElementMatrixProvider matrixProvider);

        void Clear();

        void ExtractAndInvertKii(int[] internalDofs);
        void ExtractAndInvertKiiDiagonal(int[] internalDofs);
        void ExtractKbb(int[] boundaryDofs);
        void ExtractKbiKib(int[] boundaryDofs, int[] internalDofs);

        void HandleDofOrderingWillBeModified();

        Vector MultiplyInverseKiiTimes(Vector vector);
        Matrix MultiplyInverseKiiTimes(Matrix matrix);
        Vector MultiplyInverseKiiDiagonalTimes(Vector vector);
        Matrix MultiplyInverseKiiDiagonalTimes(Matrix matrix);
        Vector MultiplyKbbTimes(Vector vector);
        Matrix MultiplyKbbTimes(Matrix matrix);
        Vector MultiplyKbiTimes(Vector vector);
        Matrix MultiplyKbiTimes(Matrix matrix);
        Vector MultiplyKibTimes(Vector vector);
        Matrix MultiplyKibTimes(Matrix matrix);
    }
}
