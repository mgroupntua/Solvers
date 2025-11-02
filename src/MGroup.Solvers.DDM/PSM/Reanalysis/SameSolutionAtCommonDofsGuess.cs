namespace MGroup.Solvers.DDM.PSM.Reanalysis
{
	using System;
	using System.Collections.Concurrent;
	using System.Collections.Generic;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.PSM.Dofs;

	/// <summary>
	/// Finds the common dofs between 2 propagation steps and uses the previous values for them, while the rest start from 0.
	/// </summary>
	public class SameSolutionAtCommonDofsGuess : IInitialSolutionGuessStrategy
	{
		private readonly IComputeEnvironment environment;
		private readonly Func<int, PsmSubdomainDofs> getSubdomainDofs;
		private readonly ReanalysisOptions reanalysisOptions;
		private readonly ConcurrentDictionary<int, IntDofTable> previousBoundaryDofOrderings = new ConcurrentDictionary<int, IntDofTable>();

		public SameSolutionAtCommonDofsGuess(IComputeEnvironment environment, ReanalysisOptions reanalysisOptions,
			Func<int, PsmSubdomainDofs> getSubdomainDofs)
		{
			this.environment = environment;
			this.getSubdomainDofs = getSubdomainDofs;
			this.reanalysisOptions = reanalysisOptions;
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessFirstSolution(
			DistributedOverlappingIndexer currentBoundaryDofIndexer)
		{
			#region log
			//Console.WriteLine("Allocating new solution vector.");
			//Debug.WriteLine("Allocating new solution vector.");
			#endregion
			StoreCurrentInterationData();
			return (new DistributedOverlappingVector(currentBoundaryDofIndexer), true);
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessNextSolution(
			DistributedOverlappingIndexer currentBoundaryDofIndexer, DistributedOverlappingVector previousSolution)
		{
			// There will be race conditions when overwriting it, but everyone will try to set it to false, 
			// thus the result will be the same.
			bool isZero = true; 

			var newSolution = new DistributedOverlappingVector(currentBoundaryDofIndexer, subdomainID =>
			{
				if (reanalysisOptions.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					// Use the previous values of the unmodified dofs. The rest will be 0.
					var currentVector = Vector.CreateZero(currentBoundaryDofIndexer.GetNumLocalIndices(subdomainID));
					Vector previousVector = previousSolution.LocalVectors[subdomainID];

					IntDofTable currentBoundaryDofs = getSubdomainDofs(subdomainID).DofOrderingBoundary;
					IntDofTable previousBoundaryDofs = previousBoundaryDofOrderings[subdomainID];
					List<int[]> commonDofs = previousBoundaryDofs.Intersect(currentBoundaryDofs);

					if (commonDofs.Count > 0)
					{
						isZero = false;
					}

					for (int i = 0; i < commonDofs.Count; ++i)
					{
						int previousIdx = commonDofs[i][2];
						int newIdx = commonDofs[i][3];
						currentVector[newIdx] = previousVector[previousIdx];
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
			return (newSolution, isZero);
		}

		private void StoreCurrentInterationData()
		{
			environment.DoPerNode(subdomainID =>
			{
				IntDofTable currentBoundaryDofs = getSubdomainDofs(subdomainID).DofOrderingBoundary.DeepCopy();
				previousBoundaryDofOrderings[subdomainID] = currentBoundaryDofs;
			});
		}
	}
}
