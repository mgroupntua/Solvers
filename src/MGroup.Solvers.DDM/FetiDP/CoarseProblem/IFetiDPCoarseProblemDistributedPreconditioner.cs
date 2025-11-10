namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;

	public interface IFetiDPCoarseProblemDistributedPreconditioner
	{
		IPreconditioner Preconditioner { get; }

		void Calculate(DistributedOverlappingIndexer cornerDofIndexer);
	}
}
