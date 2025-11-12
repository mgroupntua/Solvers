namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
	using System;
	using System.Collections.Concurrent;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.PSM.Reanalysis;
	using MGroup.Solvers.DDM.PSM.StiffnessMatrices;

	/// <summary>
	/// There is only 1 explicit matrix per subdomain, which was calculated by operations between multiple intermediate matrices,
	/// all refering to the same subdomain.
	/// </summary>
	public class PsmInterfaceProblemMatrixExplicit : IPsmInterfaceProblemMatrix
	{
		private readonly IComputeEnvironment environment;
		private readonly Func<int, IPsmSubdomainMatrixManager> getSubdomainMatrices;
		private readonly PsmReanalysisOptions reanalysis;
		private readonly ConcurrentDictionary<int, IReadOnlyMatrix> schurComplementsPerSubdomain 
			= new ConcurrentDictionary<int, IReadOnlyMatrix>();

		public PsmInterfaceProblemMatrixExplicit(IComputeEnvironment environment, 
			Func<int, IPsmSubdomainMatrixManager> getSubdomainMatrices, PsmReanalysisOptions reanalysis)
		{
			this.environment = environment;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.reanalysis = reanalysis;
		}

		public DistributedOverlappingTransformation Matrix { get; private set; }

		public void Calculate(DistributedOverlappingIndexer indexer)
		{
			//Sbb[s] = Kbb[s] - Kbi[s] * inv(Kii[s]) * Kib[s]
			Action<int> calcSchurComplement = subdomainID =>
			{
				if (!schurComplementsPerSubdomain.ContainsKey(subdomainID) 
					|| !reanalysis.SubdomainSubmatrices	|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Calculating Schur complement of internal dofs of subdomain {subdomainID}");
					//Debug.WriteLine($"Calculating Schur complement of internal dofs of subdomain {subdomainID}");
					#endregion

					IReadOnlyMatrix Sbb = getSubdomainMatrices(subdomainID).CalcSchurComplement();
					schurComplementsPerSubdomain[subdomainID] = Sbb;
				}
			};
			environment.DoPerNode(calcSchurComplement);

			Matrix = new DistributedOverlappingTransformation(indexer, MultiplySubdomainSchurComplement);
		}

		public double[] ExtractDiagonal(int subdomainID)
		{
			IReadOnlyMatrix Sbb = schurComplementsPerSubdomain[subdomainID];
			return Sbb.GetDiagonalAsArray(); //TODO: this should be a polymorphic method, the extension can be too slow
		}

		/// <summary>
		/// Sbb[s] * x = (Kbb[s] - Kbi[s] * inv(Kii[s]) * Kib[s]) * x
		/// </summary>
		/// <param name="subdomainID">The ID of a subdomain</param>
		/// <param name="input">The displacements that correspond to boundary dofs of this subdomain.</param>
		/// <param name="output">The forces that correspond to boundary dofs of this subdomain.</param>
		private void MultiplySubdomainSchurComplement(int subdomainID, Vector input, Vector output)
			=> schurComplementsPerSubdomain[subdomainID].MultiplyIntoResult(input, output);
	}
}
