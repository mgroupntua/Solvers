//TODO: Merge this with the assembler used for element -> subdomain map-reductions.
namespace MGroup.Solvers.DDM.SolversExtensions.Assemblers
{
	using System.Collections.Generic;
	using System.Diagnostics;

	using MGroup.LinearAlgebra.Matrices;

	public class DenseMatrixAssembler
	{
		public Matrix BuildGlobalMatrix(int numGlobalDofs,
			IDictionary<int, int[]> localToGlobalMaps, IDictionary<int, IMatrix> localMatrices)
		{
			var globalMatrix = Matrix.CreateZero(numGlobalDofs, numGlobalDofs);

			// Process the stiffness of each element
			foreach (var s in localToGlobalMaps.Keys)
			{
				var globalDofs = localToGlobalMaps[s];
				var localDofs = Utilities.Range(0, globalDofs.Length); //TODO: Create a DokSymmetric.AddSubmatrixSymmetric() overload that accepts a single mapping array
				AddLocalToGlobalMatrix(globalMatrix, localMatrices[s], localDofs, globalDofs);
			}

			return globalMatrix;
		}

		public void HandleDofOrderingWasModified()
		{
			// Do nothing, since there are no idexing arrays to cache.
		}

		private static void AddLocalToGlobalMatrix(Matrix globalMatrix, IReadOnlyMatrix localMatrix,
			int[] localIndices, int[] globalIndices)
		{
			Debug.Assert(localMatrix.NumRows == localMatrix.NumColumns);
			Debug.Assert(globalIndices.Length == localIndices.Length);

			var numRelevantRows = localIndices.Length;
			for (var i = 0; i < numRelevantRows; ++i)
			{
				var localRow = localIndices[i];
				var globalRow = globalIndices[i];
				for (var j = 0; j < numRelevantRows; ++j)
				{
					var localCol = localIndices[j];
					var globalCol = globalIndices[j];

					globalMatrix[globalRow, globalCol] += localMatrix[localRow, localCol];
				}
			}
		}
	}
}
