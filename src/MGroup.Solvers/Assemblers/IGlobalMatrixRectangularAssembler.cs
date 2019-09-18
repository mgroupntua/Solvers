using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.FreedomDegrees;
using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.Assemblers
{
	public interface IGlobalMatrixRectangularAssembler<TMatrix> where TMatrix:IMatrix
	{
		TMatrix BuildGlobalMatrix(ISubdomainFreeDofOrdering dofRowOrdering, ISubdomainFreeDofOrdering dofColOrdering,IEnumerable<IElement> elements,
			IElementMatrixProvider matrixProvider);

        (TMatrix matrixFreeFree, IMatrixView matrixFreeConstr, IMatrixView matrixConstrFree, IMatrixView
            matrixConstrConstr)
            BuildGlobalSubmatrices(
                ISubdomainFreeDofOrdering freeDofRowOrdering,
                ISubdomainFreeDofOrdering freeDofColOrdering,
                ISubdomainConstrainedDofOrdering constrainedDofRowOrdering,
                ISubdomainConstrainedDofOrdering constrainedDofColOrdering,
                IEnumerable<IElement> elements, IElementMatrixProvider matrixProvider);

        void HandleDofOrderingWillBeModified();
	}
}
