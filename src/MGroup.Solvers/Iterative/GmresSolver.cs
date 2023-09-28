namespace MGroup.Solvers.Iterative
{
	using System.Diagnostics;

	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.GeneralizedMinimalResidual;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.DataStructures;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers.AlgebraicModel;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;

	public class GmresSolver : SingleSubdomainSolverBase<CsrMatrix>
	{
		private readonly GmresAlgorithm gmresAlgorithm;
		private readonly IPreconditionerFactory preconditionerFactory;

		private bool mustUpdatePreconditioner = true;
		private IPreconditioner preconditioner;

		private GmresSolver(GlobalAlgebraicModel<CsrMatrix> model, GmresAlgorithm gmresAlgorithm,
			IPreconditionerFactory preconditionerFactory)
			: base(model, "GmresSolver")
		{
			this.gmresAlgorithm = gmresAlgorithm;
			this.preconditionerFactory = preconditionerFactory;
		}

		public override void HandleMatrixWillBeSet()
		{
			mustUpdatePreconditioner = true;
			preconditioner = null;
		}

		public override void Initialize() { }

		public override void PreventFromOverwrittingSystemMatrices()
		{
			// No factorization is done.
		}

		/// <summary>
		/// Solves the linear system with PCG method. If the matrix has been modified, a new preconditioner will be computed.
		/// </summary>
		public override void Solve()
		{
			var watch = new Stopwatch();
			IMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution.SingleVector == null)
			{
				LinearSystem.Solution.SingleVector = Vector.CreateZero(systemSize);
			}
			else
			{
				LinearSystem.Solution.Clear();
			}

			// Preconditioning
			if (mustUpdatePreconditioner)
			{
				watch.Start();
				preconditioner = preconditionerFactory.CreatePreconditionerFor(matrix);
				watch.Stop();
				Logger.LogTaskDuration("Calculating preconditioner", watch.ElapsedMilliseconds);
				watch.Reset();
				mustUpdatePreconditioner = false;
			}

			// Iterative algorithm
			watch.Start();
			IterativeStatistics stats = gmresAlgorithm.Solve(matrix, preconditioner, LinearSystem.RhsVector.SingleVector,
				LinearSystem.Solution.SingleVector, true, () => Vector.CreateZero(systemSize)); //TODO: This way, we don't know that x0=0, which will result in an extra b-A*0
			if (!stats.HasConverged)
			{
				throw new IterativeSolverNotConvergedException(Name + " did not converge to a solution. PCG algorithm run for"
					+ $" {stats.NumIterationsRequired} iterations and the residual norm ratio was"
					+ $" {stats.ResidualNormRatioEstimation}");
			}
			watch.Stop();
			Logger.LogTaskDuration("Iterative algorithm", watch.ElapsedMilliseconds);
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			Logger.IncrementAnalysisStep();
		}

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
		{
			//TODO: Use a reorthogonalizetion approach when solving multiple rhs vectors. It would be even better if the CG
			//      algorithm exposed a method for solving for multiple rhs vectors.
			var watch = new Stopwatch();

			// Preconditioning
			IMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			if (mustUpdatePreconditioner)
			{
				watch.Start();
				preconditioner = preconditionerFactory.CreatePreconditionerFor(matrix);
				watch.Stop();
				Logger.LogTaskDuration("Calculating preconditioner", watch.ElapsedMilliseconds);
				watch.Reset();
				mustUpdatePreconditioner = false;
			}

			// Iterative algorithm
			watch.Start();
			int numRhs = otherMatrix.NumColumns;
			var solutionVectors = Matrix.CreateZero(systemSize, numRhs);
			Vector solutionVector = Vector.CreateZero(systemSize);

			// Solve each linear system
			for (int j = 0; j < numRhs; ++j)
			{
				if (j != 0)
				{
					solutionVector.Clear();
				}

				//TODO: we should make sure this is the same type as the vectors used by this solver, otherwise vector operations
				//      in GMRES will be slow.
				Vector rhsVector = otherMatrix.GetColumn(j);

				IterativeStatistics stats = gmresAlgorithm.Solve(matrix, preconditioner, rhsVector,
					solutionVector, true, () => Vector.CreateZero(systemSize));

				solutionVectors.SetSubcolumn(j, solutionVector);
			}

			watch.Stop();
			Logger.LogTaskDuration("Iterative algorithm", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
			return solutionVectors;
		}

		public class Factory
		{
			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public GmresAlgorithm GmresAlgorithm { get; set; } = new GmresAlgorithm.Builder().Build();

			public IPreconditionerFactory PreconditionerFactory { get; set; } = new IdentityPreconditioner.Factory();

			public GmresSolver BuildSolver(GlobalAlgebraicModel<CsrMatrix> model)
				=> new GmresSolver(model, GmresAlgorithm, PreconditionerFactory);

			public GlobalAlgebraicModel<CsrMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<CsrMatrix>(model, DofOrderer, new CsrMatrixAssembler(true));
		}
	}
}
