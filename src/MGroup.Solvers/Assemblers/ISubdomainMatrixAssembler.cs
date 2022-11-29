using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Providers;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.DofOrdering;

//TODO: not sure this interface is required
namespace MGroup.Solvers.Assemblers
{
    /// <summary>
    /// Builds the matrix of the linear system that will be solved.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    /// <typeparam name="TMatrix">The type of the matrix that will be solved.</typeparam>
    public interface ISubdomainMatrixAssembler<TMatrix>
        where TMatrix : class, IMatrix
    {
		TMatrix CreateEmptyMatrix(ISubdomainFreeDofOrdering dofOrdering);
		
		/// <summary>
		/// Builds the linear system matrix that corresponds to the free freedom degrees of a subdomain.
		/// </summary>
		/// <param name="dofOrdering">The free freedom degree ordering of the subdomain.</param>
		/// <param name="elements">The (finite) elements of the subdomain.</param>
		/// <param name="matrixProvider">Determines the matrix calculated for each element (e.g. stiffness, mass, etc.)</param>
		TMatrix BuildGlobalMatrix(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElementType> elements,
            IElementMatrixProvider matrixProvider);

		ISubdomainMatrixAssembler<TMatrix> Clone();

		/// <summary>
		/// Update internal state when the freedom degree ordering is changed (e.g. reordering, XFEM, adaptive FEM). It 
		/// will be called after modifying the current freedom degree ordering.
		/// </summary>
		void HandleDofOrderingWasModified();
    }

	public static class ISubdomainMatrixAssemblerExtensions
	{
		public static TMatrix RebuildSubdomainMatrix<TMatrix>(this ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler, 
			IEnumerable<IElementType> subdomainElements, ISubdomainFreeDofOrdering subdomainDofs, 
			IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate)
			where TMatrix : class, IMatrix
		{
			bool rebuildSubdomainMatrix = false;
			foreach (IElementType element in subdomainElements)
			{
				if (predicate.MustBuildMatrixForElement(element))
				{
					rebuildSubdomainMatrix = true;
					break;
				}
			}

			if (rebuildSubdomainMatrix)
			{
				TMatrix matrix = subdomainMatrixAssembler.BuildGlobalMatrix(
					subdomainDofs, subdomainElements, elementMatrixProvider);
				foreach (IElementType element in subdomainElements)
				{
					predicate.ProcessElementAfterBuildingMatrix(element);
				}
				return matrix;
			}
			else
			{
				foreach (IElementType element in subdomainElements)
				{
					predicate.ProcessElementAfterNotBuildingMatrix(element);
				}
				return null;
			}
		}
	}
}
