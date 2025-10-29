namespace MGroup.Solvers.DDM.FetiDP.Reanalysis
{
	using System.Collections.Concurrent;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.LagrangeMultipliers;

	/// <summary>
	/// Finds the common dofs between 2 propagation steps and uses the previous values for them, while the rest start from 0.
	/// </summary>
	public class SameSolutionAtCommonDofsGuess : IInitialSolutionGuessStrategy
	{
		private readonly IComputeEnvironment environment;
		private readonly Func<int, SubdomainLagranges> getSubdomainLagranges;
		private readonly ReanalysisOptions reanalysisOptions;
		private readonly ConcurrentDictionary<int, SubdomainLagrangeData> previousLagrangeData 
			= new ConcurrentDictionary<int, SubdomainLagrangeData>();

		public SameSolutionAtCommonDofsGuess(IComputeEnvironment environment, ReanalysisOptions reanalysisOptions,
			Func<int, SubdomainLagranges> getSubdomainLagranges)
		{
			this.environment = environment;
			this.getSubdomainLagranges = getSubdomainLagranges;
			this.reanalysisOptions = reanalysisOptions;
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessFirstSolution(
			DistributedOverlappingIndexer currentLagrangeVectorIndexer)
		{
			#region log
			//Console.WriteLine("Allocating new solution vector.");
			//Debug.WriteLine("Allocating new solution vector.");
			#endregion
			StoreCurrentInterationData();
			return (new DistributedOverlappingVector(currentLagrangeVectorIndexer), true);
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessNextSolution(
			DistributedOverlappingIndexer currentLagrangeVectorIndexer, DistributedOverlappingVector previousSolution)
		{
			// There will be race conditions when overwriting it, but everyone will try to set it to false, 
			// thus the result will be the same.
			bool isZero = true;

			var newSolution = new DistributedOverlappingVector(currentLagrangeVectorIndexer, subdomainID =>
			{
				if (reanalysisOptions.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					// Use the previous values of the unmodified dofs. The rest will be 0.
					var currentVector = Vector.CreateZero(currentLagrangeVectorIndexer.GetNumLocalIndices(subdomainID));
					Vector previousVector = previousSolution.LocalVectors[subdomainID];

					List<LagrangeMultiplier> currentLagranges = getSubdomainLagranges(subdomainID).LagrangeMultipliers;
					SubdomainLagrangeData previousLagranges = previousLagrangeData[subdomainID];

					foreach (LagrangeMultiplier lagr in currentLagranges)
					{
						bool existedPreviously = previousLagranges.TryGetIndexOfLagrangeMultiplier(
							lagr.NodeID, lagr.DofID, lagr.SubdomainPlus, lagr.SubdomainMinus, out int previousIdx);
						if (existedPreviously)
						{
							currentVector[lagr.LocalIdx] = previousVector[previousIdx];
							isZero = false;
						}
					}
					return currentVector;
				}
				else
				{
					isZero = false;
					// If the subdomain has the same dofs as before, reuse the vector as it is
					#region log
					//Console.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
					//Debug.WriteLine($"Reusing the previous solution subvector for subdomain {subdomainID}.");
					#endregion
					return previousSolution.LocalVectors[subdomainID];
				}
			});

			StoreCurrentInterationData();
			#region debug
			//double tol = 1E-7;
			//bool check = newSolution.AreOverlappingEntriesEqual(tol);
			#endregion
			return (newSolution, isZero);
		}

		private void StoreCurrentInterationData()
		{
			environment.DoPerNode(subdomainID =>
			{
				List<LagrangeMultiplier> currentLagranges = getSubdomainLagranges(subdomainID).LagrangeMultipliers;
				previousLagrangeData[subdomainID] = new SubdomainLagrangeData(currentLagranges);
			});
		}

		private class SubdomainLagrangeData
		{
			/// <summary>
			/// Key 0: nodeID. Key 1: dofID. Key 2: subdomainPlusID. Key 3: subdomainMinusID. 
			/// Value: index in lagranges vector of subdomain.
			/// </summary>
			//		                 nodeID           dofID   subdomainPlus  subdomainMinus  vectorIdx
			private readonly Dictionary<int, Dictionary<int, Dictionary<int, Dictionary<int, int>>>> data;

			public SubdomainLagrangeData(List<LagrangeMultiplier> lagrangeMultipliersOfSubdomain)
			{
				data = new Dictionary<int, Dictionary<int, Dictionary<int, Dictionary<int, int>>>>();
				foreach (LagrangeMultiplier lagr in lagrangeMultipliersOfSubdomain)
				{
					bool nodeExists = data.TryGetValue(lagr.NodeID, out var dataOfNode);
					if (!nodeExists)
					{
						dataOfNode = new Dictionary<int, Dictionary<int, Dictionary<int, int>>>();
						data[lagr.NodeID] = dataOfNode;
					}

					bool dofExists = dataOfNode.TryGetValue(lagr.DofID, out var dataOfDof);
					if (!dofExists)
					{
						dataOfDof = new Dictionary<int, Dictionary<int, int>>();
						dataOfNode[lagr.DofID] = dataOfDof;
					}

					bool subPlusExists = dataOfDof.TryGetValue(lagr.SubdomainPlus, out var dataOfSubPlus);
					if (!subPlusExists)
					{
						dataOfSubPlus = new Dictionary<int, int>();
						dataOfDof[lagr.SubdomainPlus] = dataOfSubPlus;
					}

					dataOfSubPlus[lagr.SubdomainMinus] = lagr.LocalIdx;
				}
			}

			public bool TryGetIndexOfLagrangeMultiplier(int nodeID, int dofID, int subdomainPlus, int subdomainMinus, out int index)
			{
				bool nodeExists = data.TryGetValue(nodeID, out var dataOfNode);
				if (nodeExists)
				{
					bool dofExists = dataOfNode.TryGetValue(dofID, out var dataOfDof);
					if (dofExists)
					{
						bool subPlusExists = dataOfDof.TryGetValue(subdomainPlus, out var dataOfSubPlus);
						if (subPlusExists)
						{
							bool subMinusExists = dataOfSubPlus.TryGetValue(subdomainMinus, out int localIndex);
							if (subMinusExists)
							{
								index = localIndex;
								return true;
							}
						}
					}
				}

				// If any of the above is not stored
				index = -1;
				return false;
			}
		}
	}
}
