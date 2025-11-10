using System.Diagnostics;

using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient.BlockPcg;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.LinearSystem;

namespace MGroup.Solvers.Iterative
{
	/// <summary>
	/// Iterative solver for models with only 1 subdomain. Uses the Block Proconditioned Conjugate Gradient algorithm.
	/// </summary>
	public class BlockPcgSolver : SingleSubdomainSolverBase<CsrMatrix>
	{
		private readonly BlockPcgAlgorithm blockPcgAlgorithm;
		private readonly bool matrixPatternWillNotBeModified;
		private readonly IPreconditioner preconditioner;

		private bool mustUpdatePreconditioner = true;

		private BlockPcgSolver(GlobalAlgebraicModel<CsrMatrix> model, BlockPcgAlgorithm blockPcgAlgorithm,
			IPreconditioner preconditioner, bool matrixPatternWillNotBeModified)
			: base(model, "BlockPcgSolver")
		{
			this.blockPcgAlgorithm = blockPcgAlgorithm;
			this.matrixPatternWillNotBeModified = matrixPatternWillNotBeModified;
			this.preconditioner = preconditioner;
		}

		public override void HandleMatrixWillBeSet()
		{
			mustUpdatePreconditioner = true;
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

			IMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution == null)
			{
				LinearSystem.Solution = Vector.CreateZero(systemSize);
			}
			else LinearSystem.Solution.Clear();

			// Preconditioning
			if (mustUpdatePreconditioner)
			{
				watch.Start();
				preconditioner.UpdateMatrix(matrix, !matrixPatternWillNotBeModified);
				watch.Stop();
				Logger.LogTaskDuration("Calculating preconditioner", watch.ElapsedMilliseconds);
				watch.Reset();
				mustUpdatePreconditioner = false;
			}

			// Iterative algorithm
			watch.Start();
			IterativeStatistics stats = blockPcgAlgorithm.Solve(matrix, preconditioner,
				LinearSystem.RhsVector, LinearSystem.Solution,
				true); //TODO: This way, we don't know that x0=0, which will result in an extra b-A*0
			if (!stats.HasConverged)
			{
				throw new IterativeSolverNotConvergedException(Name + " did not converge to a solution. BlockPCG algorithm run for"
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
			IMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			if (mustUpdatePreconditioner)
			{
				watch.Start();
				preconditioner.UpdateMatrix(matrix, !matrixPatternWillNotBeModified);
				watch.Stop();
				Logger.LogTaskDuration("Calculating preconditioner", watch.ElapsedMilliseconds);
				watch.Reset();
				mustUpdatePreconditioner = false;
			}

			// Iterative algorithm
			watch.Start();
			int numRhs = otherMatrix.NumColumns;
			var solutionVectors = Matrix.CreateZero(systemSize, numRhs);
			var solutionVector = Vector.CreateZero(systemSize);

			// Solve each linear system
			for (int j = 0; j < numRhs; ++j)
			{
				if (j != 0) solutionVector.Clear();

				//TODO: we should make sure this is the same type as the vectors used by this solver, otherwise vector operations
				//      in CG will be slow.
				Vector rhsVector = otherMatrix.GetColumn(j);

				IterativeStatistics stats = blockPcgAlgorithm.Solve(matrix, preconditioner, rhsVector, solutionVector, true);

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

			public BlockPcgAlgorithm BlockPcgAlgorithm { get; set; } = (new BlockPcgAlgorithm.Factory()).Build();

			public bool MatrixPatternWillNotBeModified { get; set; } = false;

			public IPreconditioner Preconditioner { get; set; } = new JacobiPreconditioner();

			public BlockPcgSolver BuildSolver(GlobalAlgebraicModel<CsrMatrix> model)
				=> new BlockPcgSolver(model, BlockPcgAlgorithm, Preconditioner.CopyWithInitialSettings(),
					MatrixPatternWillNotBeModified);

			public GlobalAlgebraicModel<CsrMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<CsrMatrix>(model, DofOrderer, new CsrMatrixAssembler(true));
		}
	}
}
