using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.ConjugateGradient;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.LinearSystem;

namespace MGroup.Solvers.Iterative
{
	/// <summary>
	/// Iterative solver for models with only 1 subdomain. Uses the Proconditioned Conjugate Gradient algorithm.
	/// </summary>
	public class PcgSolver : SingleSubdomainSolverBase<CsrMatrix>
	{
		private readonly PcgAlgorithm pcgAlgorithm;
		private readonly bool matrixPatternWillNotBeModified;
		private readonly IPreconditioner preconditioner;

		private bool mustUpdatePreconditioner = true;

		private PcgSolver(GlobalAlgebraicModel<CsrMatrix> model, PcgAlgorithm pcgAlgorithm,
			IPreconditioner preconditioner, bool matrixPatternWillNotBeModified)
			: base(model, "PcgSolver")
		{
			this.pcgAlgorithm = pcgAlgorithm;
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
			IterativeStatistics stats = pcgAlgorithm.Solve(matrix, preconditioner,
				LinearSystem.RhsVector, LinearSystem.Solution,
				true); //TODO: This way, we don't know that x0=0, which will result in an extra b-A*0
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

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IReadOnlyMatrix otherMatrix)
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

				IterativeStatistics stats = pcgAlgorithm.Solve(matrix, preconditioner, rhsVector,
					solutionVector, true);

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

			public bool MatrixPatternWillNotBeModified { get; set; } = false;

			public PcgAlgorithm PcgAlgorithm { get; set; } = (new PcgAlgorithm.Factory()).Build();

			public IPreconditioner Preconditioner { get; set; } = new JacobiPreconditioner();

			public PcgSolver BuildSolver(GlobalAlgebraicModel<CsrMatrix> model)
				=> new PcgSolver(model, PcgAlgorithm, Preconditioner.CopyWithInitialSettings(), MatrixPatternWillNotBeModified);

			public GlobalAlgebraicModel<CsrMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<CsrMatrix>(model, DofOrderer, new CsrMatrixAssembler(true));
		}
	}
}
