namespace MGroup.Solvers.Direct
{
	using System.Diagnostics;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Triangulation;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers.AlgebraicModel;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;

	public class CSparseLUSolver : SingleSubdomainSolverBase<CscMatrix>
	{
		private readonly double factorizationPivotTolerance;

		private bool factorizeInPlace = true;
		private bool mustFactorize = true;
		private LUCSparseNet factorization;

		private CSparseLUSolver(GlobalAlgebraicModel<CscMatrix> model, double factorizationPivotTolerance)
			: base(model, "CSparseLUSolver")
		{
			this.factorizationPivotTolerance = factorizationPivotTolerance;
		}

		public override void HandleMatrixWillBeSet()
		{
			mustFactorize = true;
			factorization = null;
		}

		public override void Initialize() { }

		public override void PreventFromOverwrittingSystemMatrices() => factorizeInPlace = false;

		/// <summary>
		/// Solves the linear system with back-forward substitution. If the matrix has been modified, it will be refactorized.
		/// </summary>
		public override void Solve()
		{
			var watch = new Stopwatch();
			CscMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution.SingleVector == null)
			{
				LinearSystem.Solution.SingleVector = Vector.CreateZero(systemSize);
			}
			//else linearSystem.Solution.Clear(); // no need to waste computational time on this in a direct solver

			// Factorization
			if (mustFactorize)
			{
				watch.Start();
				factorization = LUCSparseNet.Factorize(matrix, factorizationPivotTolerance);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();
			factorization.SolveLinearSystem(LinearSystem.RhsVector.SingleVector, LinearSystem.Solution.SingleVector);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
		}

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
		{
			var watch = new Stopwatch();

			// Factorization
			CscMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			if (mustFactorize)
			{
				watch.Start();
				factorization = LUCSparseNet.Factorize(matrix, factorizationPivotTolerance);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();

			// Solve each linear system separately, to avoid copying the RHS matrix to a dense one.
			int systemOrder = matrix.NumColumns;
			int numRhs = otherMatrix.NumColumns;
			var solutionVectors = Matrix.CreateZero(systemOrder, numRhs);
			Vector solutionVector = Vector.CreateZero(systemSize);
			for (int j = 0; j < numRhs; ++j)
			{
				if (j != 0) solutionVector.Clear();
				Vector rhsVector = otherMatrix.GetColumn(j);
				factorization.SolveLinearSystem(rhsVector, solutionVector);
				solutionVectors.SetSubcolumn(j, solutionVector);
			}

			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
			return solutionVectors;
		}

		public class Factory
		{
			public Factory() { }

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), AmdReordering.CreateWithCSparseAmd());

			public double FactorizationPivotTolerance { get; set; } = 1E-15;

			public CSparseLUSolver BuildSolver(GlobalAlgebraicModel<CscMatrix> model)
			{
				return new CSparseLUSolver(model, FactorizationPivotTolerance);
			}

			public GlobalAlgebraicModel<CscMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<CscMatrix>(model, DofOrderer, new CscMatrixAssembler(false, true));
		}
	}
}
