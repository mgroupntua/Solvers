namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
	using System.Collections.Generic;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.PSM.Vectors;

	public class PsmInterfaceProblemVectorsReanalysis : IPsmInterfaceProblemVectors
	{
		private const bool cacheDistributedVectorBuffers = true;
		private readonly IComputeEnvironment environment;
		private readonly IModifiedSubdomains modifiedSubdomains;
		private DistributedOverlappingVector previousCondensedFbVectors;
		private readonly IDictionary<int, PsmSubdomainVectors> subdomainVectors;

		public PsmInterfaceProblemVectorsReanalysis(IComputeEnvironment environment, 
			IDictionary<int, PsmSubdomainVectors> subdomainVectors, IModifiedSubdomains modifiedSubdomains)
		{
			this.environment = environment;
			this.subdomainVectors = subdomainVectors;
			this.modifiedSubdomains = modifiedSubdomains;
		}

		public DistributedOverlappingVector InterfaceProblemRhs { get; private set; }

		public DistributedOverlappingVector InterfaceProblemSolution { get; set; }

		// globalF = sum {Lb[s]^T * (fb[s] - Kbi[s] * inv(Kii[s]) * fi[s]) }
		public void CalcInterfaceRhsVector(DistributedOverlappingIndexer indexer)
		{
			bool isFirstAnalysis = previousCondensedFbVectors == null;
			Dictionary<int, Vector> fbCondensed = environment.CalcNodeData(subdomainID =>
			{
				if (isFirstAnalysis || modifiedSubdomains.IsRhsModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Calculating condensed rhs of subdomain {subdomainID}");
					//Debug.WriteLine($"Calculating condensed rhs of subdomain {subdomainID}");
					#endregion

					return subdomainVectors[subdomainID].CalcCondensedRhsVector();
				}
				else
				{
					return previousCondensedFbVectors.LocalVectors[subdomainID];
				}
			});

			InterfaceProblemRhs = new DistributedOverlappingVector(indexer, fbCondensed);
			InterfaceProblemRhs.CacheSendRecvBuffers = cacheDistributedVectorBuffers;
			previousCondensedFbVectors = InterfaceProblemRhs.CopyAsDistributed();
			InterfaceProblemRhs.SumOverlappingEntries();
		}

		public void Clear()
		{
			InterfaceProblemRhs = null;
			previousCondensedFbVectors = null;
		}
	}
}
