namespace MGroup.Solvers.DDM.PSM.Preconditioning
{
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;

	public class PsmPreconditionerIdentity : IPsmPreconditioner
	{

		public PsmPreconditionerIdentity()
		{
		}

		public IPreconditioner Preconditioner { get; private set; }

		public void Calculate(IComputeEnvironment environment, DistributedOverlappingIndexer boundaryDofIndexer,
			IPsmInterfaceProblemMatrix interfaceProblemMatrix) 
		{
			Preconditioner = new IdentityPreconditioner();
		}
	}
}
