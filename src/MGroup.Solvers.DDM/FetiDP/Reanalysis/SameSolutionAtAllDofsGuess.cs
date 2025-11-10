namespace MGroup.Solvers.DDM.FetiDP.Reanalysis
{
	using MGroup.LinearAlgebra.Distributed.Overlapping;

	/// <summary>
	/// Will reuse the previous solution if ALL dofs are the same between the 2 iterations.
	/// </summary>
	public class SameSolutionAtAllDofsGuess : IInitialSolutionGuessStrategy
	{

		public SameSolutionAtAllDofsGuess()
		{
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessFirstSolution(
			DistributedOverlappingIndexer currentLagrangeVectorIndexer)
		{
			#region log
			//Console.WriteLine("Allocating new solution vector.");
			//Debug.WriteLine("Allocating new solution vector.");
			#endregion
			return (new DistributedOverlappingVector(currentLagrangeVectorIndexer), true);
		}

		public (DistributedOverlappingVector guess, bool isZero) GuessNextSolution(
			DistributedOverlappingIndexer currentLagrangeVectorIndexer, DistributedOverlappingVector previousSolution)
		{
			if (currentLagrangeVectorIndexer.IsCompatibleWith(previousSolution.Indexer))
			{
				// Reuse the existing whole vector.
				//TODO: Won't this cause problems with references? Should I just copy the subvectors to a new one? 
				#region log
				//Console.WriteLine("Reusing the previous solution vector.");
				//Debug.WriteLine("Reusing the previous solution vector.");
				#endregion
				return (previousSolution, false);
			}
			else
			{
				return (new DistributedOverlappingVector(currentLagrangeVectorIndexer), true);
			}
		}
	}
}
