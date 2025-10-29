namespace MGroup.Solvers.DDM.PSM.Preconditioning
{
	using System;
	using System.Collections.Generic;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;

	public class PsmPreconditionerJacobi : IPsmPreconditioner
	{
		public PsmPreconditionerJacobi()
		{
		}

		public IPreconditioner Preconditioner { get; private set; }

		public void Calculate(IComputeEnvironment environment, DistributedOverlappingIndexer boundaryDofIndexer,
			IPsmInterfaceProblemMatrix interfaceProblemMatrix)
		{
			Func<int, Vector> extractDiagonal = subdomainID
				=> Vector.CreateFromArray(interfaceProblemMatrix.ExtractDiagonal(subdomainID));
			Dictionary<int, Vector> localDiagonals = environment.CalcNodeData(extractDiagonal);
			var distributedDiagonal = new DistributedOverlappingVector(boundaryDofIndexer, localDiagonals);

			// All dofs belong to 2 or more subdomains and must have the stiffness contributions from all these subdomains.
			distributedDiagonal.SumOverlappingEntries();

			this.Preconditioner = new DistributedOverlappingJacobiPreconditioner(environment, distributedDiagonal);
		}
	}
}
