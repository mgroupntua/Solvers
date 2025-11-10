namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

	public class FetiDPCoarseProblemDistributedPreconditionerJacobi : IFetiDPCoarseProblemDistributedPreconditioner
	{
		private readonly IComputeEnvironment environment;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;

		public FetiDPCoarseProblemDistributedPreconditionerJacobi(IComputeEnvironment environment,
			Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
		{
			this.environment = environment;
			this.getSubdomainMatrices = getSubdomainMatrices;
		}

		public IPreconditioner Preconditioner { get; private set; }

		public void Calculate(DistributedOverlappingIndexer cornerDofIndexer)
		{
			Func<int, Vector> extractDiagonal = subdomainID
				=> getSubdomainMatrices(subdomainID).SchurComplementOfRemainderDofs.GetDiagonal();
			Dictionary<int, Vector> localDiagonals = environment.CalcNodeData(extractDiagonal);
			var distributedDiagonal = new DistributedOverlappingVector(cornerDofIndexer, localDiagonals);

			// All dofs belong to 2 or more subdomains and must have the stiffness contributions from all these subdomains.
			distributedDiagonal.SumOverlappingEntries();

			this.Preconditioner = new DistributedOverlappingJacobiPreconditioner(environment, distributedDiagonal);
		}
	}
}
