namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;

	public class FetiDPCoarseProblemDistributedPreconditionerIdentity : IFetiDPCoarseProblemDistributedPreconditioner
	{
		public IPreconditioner Preconditioner { get; } = new IdentityPreconditioner();

		public void Calculate(DistributedOverlappingIndexer cornerDofIndexer) { }
	}
}
