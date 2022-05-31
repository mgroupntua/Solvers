using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Builders;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Providers;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers.Assemblers
{
	/// <summary>
	/// Builds the global matrix of the linear system that will be solved. This matrix is in symmetric DOK format, namely only 
	/// the upper triangle is explicitly stored.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	public class SymmetricDokMatrixAssembler /*: ISubdomainMatrixAssembler<SymmetricCscMatrix>*/
	{
		private const string name = "SymmetricDokMatrixAssembler"; // for error messages

		public SymmetricDokMatrixAssembler()
		{
		}

		public DokSymmetric BuildGlobalMatrix(ISubdomainFreeDofOrdering dofOrdering, IEnumerable<IElementType> elements,
			IElementMatrixProvider matrixProvider)
		{
			int numFreeDofs = dofOrdering.NumFreeDofs;
			var subdomainMatrix = DokSymmetric.CreateEmpty(numFreeDofs);

			foreach (IElementType element in elements)
			{
				// TODO: perhaps that could be done and cached during the dof enumeration to avoid iterating over the dofs twice
				(int[] elementDofIndices, int[] subdomainDofIndices) = dofOrdering.MapFreeDofsElementToSubdomain(element);
				IMatrix elementMatrix = matrixProvider.Matrix(element);
				subdomainMatrix.AddSubmatrixSymmetric(elementMatrix, elementDofIndices, subdomainDofIndices);
			}

			return subdomainMatrix;
		}

		public SymmetricDokMatrixAssembler Clone() => new SymmetricDokMatrixAssembler();

		public void HandleDofOrderingWasModified()
		{
			//TODO: Implement this
		}
	}
}
