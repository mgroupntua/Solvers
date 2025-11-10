namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	using System.Diagnostics;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.Output;

	public class FetiDPCoarseProblemDistributed : IFetiDPCoarseProblem
	{
		private readonly IComputeEnvironment environment;
		private readonly ISubdomainTopology subdomainTopology;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;
		private readonly IFetiDPCoarseProblemDistributedPreconditioner coarseProblemPreconditioner;
		private readonly ISystemSolutionIterativeMethod coarseProblemSolver;
		//private readonly bool areSchurComplementsExplicit;

		private DistributedOverlappingIndexer cornerDofIndexer;
		private DistributedOverlappingTransformation coarseProblemMatrix;

		public FetiDPCoarseProblemDistributed(IComputeEnvironment environment, ISubdomainTopology subdomainTopology,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices,
			ISystemSolutionIterativeMethod coarseProblemSolver, bool useJacobiPreconditioner/*, bool areSchurComplementsExplicit*/)
		{
			this.environment = environment;
			this.subdomainTopology = subdomainTopology;
			this.getSubdomainDofs = getSubdomainDofs;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.coarseProblemSolver = coarseProblemSolver;
			//this.areSchurComplementsExplicit = areSchurComplementsExplicit;

			if (useJacobiPreconditioner)
			{
				coarseProblemPreconditioner = new FetiDPCoarseProblemDistributedPreconditionerJacobi(
					environment, getSubdomainMatrices);
				//this.areSchurComplementsExplicit = true;
			}
			else
			{
				coarseProblemPreconditioner = new FetiDPCoarseProblemDistributedPreconditionerIdentity();
			}
		}

		public void FindCoarseProblemDofs(DdmLogger logger, IModifiedCornerDofs modifiedCornerDofs)
		{
			cornerDofIndexer = subdomainTopology.CreateDistributedVectorIndexer(s => getSubdomainDofs(s).DofOrderingCorner);
			coarseProblemSolver.Clear();

			if (logger != null)
			{
				logger.LogProblemSize(2, cornerDofIndexer.NumGlobalIndices);
			}
		}

		public void PrepareMatricesForSolution()
		{
			//if (areSchurComplementsExplicit)
			//{
				//environment.DoPerNode(s => getSubdomainMatrices(s).CalcSchurComplementOfRemainderDofs()); // this is done by the solver
				coarseProblemMatrix = new DistributedOverlappingTransformation(cornerDofIndexer,
					(s, vIn, vOut) => getSubdomainMatrices(s).SchurComplementOfRemainderDofs.MultiplyIntoResult(vIn, vOut));
			//}
			//else
			//{
			//	coarseProblemMatrix = new DistributedOverlappingTransformation(cornerDofIndexer,
			//		(s, vIn, vOut) => getSubdomainMatrices(s).MultiplySchurComplementImplicitly(vIn, vOut));
			//}
			coarseProblemPreconditioner.Calculate(cornerDofIndexer);
		}

		public void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			var distributedRhs = new DistributedOverlappingVector(cornerDofIndexer, coarseProblemRhs);
			var distributedSolution = new DistributedOverlappingVector(cornerDofIndexer, coarseProblemSolution);

			distributedRhs.SumOverlappingEntries();

			IterativeStatistics stats = coarseProblemSolver.Solve(
				coarseProblemMatrix, coarseProblemPreconditioner.Preconditioner, distributedRhs, distributedSolution, true);
			Debug.WriteLine($"Coarse problem solution: iterations = {stats.NumIterationsRequired}, " +
				$"residual norm ratio = {stats.ResidualNormRatioEstimation}");
			//MpiUtilities.DeclareRootOnlyProcess("Coarse problem solution: iterations = {stats.NumIterationsRequired}, " +
			//	$"residual norm ratio = {stats.ResidualNormRatioEstimation}");
		}

		public class Factory : IFetiDPCoarseProblemFactory
		{
			public Factory()
			{
				//AreSchurComplementsExplicit = true;
				UseJacobiPreconditioner = true;

				var pcgFactory = new PcgAlgorithm.Factory();
				pcgFactory.ResidualTolerance = 1E-6;
				pcgFactory.MaxIterationsProvider = new FixedMaxIterationsProvider(100);
				CoarseProblemSolver = pcgFactory.Build();
			}

			//public bool AreSchurComplementsExplicit { get; set; }

			public ISystemSolutionIterativeMethod CoarseProblemSolver { get; set; }

			public bool UseJacobiPreconditioner { get; set; }

			public IFetiDPCoarseProblem CreateCoarseProblem(
				IComputeEnvironment environment, ISubdomainTopology subdomainTopology, 
				Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
			{
				return new FetiDPCoarseProblemDistributed(environment, subdomainTopology, getSubdomainDofs,
					getSubdomainMatrices, CoarseProblemSolver, UseJacobiPreconditioner/*, AreSchurComplementsExplicit*/);
			}
		}
	}
}
