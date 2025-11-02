namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
	using System;
	using System.Diagnostics;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.PSM.Dofs;
	using MGroup.Solvers.DDM.PSM.StiffnessMatrices;

	/// <summary>
	/// Each subdomain's matrix is implicitly represented by an expression invovling multiple intermediate matrices, 
	/// all refering to the same subdomain. Taking advantage of the distributed property of matrix-vector multiplication, 
	/// multiplying each subdomain's "matrix" with a vector is performed by multiplying the intermediate matrices with that 
	/// vector and reducing the intermediate vector results.
	/// </summary>
	public class PsmInterfaceProblemMatrixImplicit : IPsmInterfaceProblemMatrix
	{
		private readonly IComputeEnvironment environment;
		private readonly Func<int, PsmSubdomainDofs> getSubdomainDofs;
		private readonly Func<int, IPsmSubdomainMatrixManager> getSubdomainMatrices;

		public PsmInterfaceProblemMatrixImplicit(IComputeEnvironment environment, Func<int, PsmSubdomainDofs> getSubdomainDofs, 
			Func<int, IPsmSubdomainMatrixManager> getSubdomainMatrices)
		{
			this.environment = environment;
			this.getSubdomainDofs = getSubdomainDofs;
			this.getSubdomainMatrices = getSubdomainMatrices;
		}

		public DistributedOverlappingTransformation Matrix { get; private set; }

		public void Calculate(DistributedOverlappingIndexer indexer)
		{
			Matrix = new DistributedOverlappingTransformation(indexer, MultiplySubdomainSchurComplement);
		}

		//TODO: this looks like it belongs to LinearAlgebra. Perhaps in an implicit matrix class.
		//TODO: There should be a way to learn the number of boundary dofs from Kbb, without depending on DofSeparator
		//TODO: This is extremely inefficient. Forming Sbb explicitly should be faster, especially since it is done by the 
		//		MatrixManager classes, which have access to the concrete type of the submatrices. And then matrix vector 
		//		multiplications during PCG will be much faster. Perhaps Sbb * vector should be delegated to MatrixManager, which
		//		will do it implicitly, unless CalcSchurComplement was called.
		public double[] ExtractDiagonal(int subdomainID) 
		{
			// Multiply with the columns of identity matrix and keep the corresponding entries.
			int numBoundaryDofs = getSubdomainDofs(subdomainID).DofsBoundaryToFree.Length; 
			var lhs = Vector.CreateZero(numBoundaryDofs);
			var rhs = Vector.CreateZero(numBoundaryDofs);
			var diagonal = new double[numBoundaryDofs];
			for (int j = 0; j < numBoundaryDofs; ++j)
			{
				// Lhs vector is a column of the identity matrix
				if (j > 0)
				{
					lhs.Clear();
				}
				lhs[j] = 1.0;

				// Multiply Sbb * lh
				MultiplySubdomainSchurComplement(subdomainID, lhs, rhs);

				// Keep the entry that corresponds to the diagonal.
				diagonal[j] = rhs[j];
			}

			return diagonal;
		}

		/// <summary>
		/// S[s] * x = (Kbb[s] - Kbi[s] * inv(Kii[s]) * Kib[s]) * x
		/// </summary>
		/// <param name="subdomainID">The ID of a subdomain</param>
		/// <param name="input">The displacements that correspond to boundary dofs of this subdomain.</param>
		/// <param name="output">The forces that correspond to boundary dofs of this subdomain.</param>
		private void MultiplySubdomainSchurComplement(int subdomainID, Vector input, Vector output)
		{
			getSubdomainMatrices(subdomainID).MultiplySchurComplementImplicitly(input, output);
		}
	}
}
