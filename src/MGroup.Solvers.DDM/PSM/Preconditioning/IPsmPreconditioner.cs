namespace MGroup.Solvers.DDM.PSM.Preconditioning
{
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;

	public interface IPsmPreconditioner
	{
		void Calculate(IComputeEnvironment environment, DistributedOverlappingIndexer boundaryDofIndexer, 
			IPsmInterfaceProblemMatrix interfaceProblemMatrix);

		IPreconditioner Preconditioner { get; }
	}
}
