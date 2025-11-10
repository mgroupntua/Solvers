namespace MGroup.Solvers.Direct
{
	using System.Diagnostics;

	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Implementations.Managed;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Triangulation;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;
	using MGroup.Solvers.LinearSystem;

	public class LUCscSolver : SingleSubdomainSolverBase<CscMatrix>
	{
		private readonly IImplementationProvider provider;

		private bool factorizeInPlace = true;
		private bool mustFactorize = true;
		private ILUCscFactorization factorization;

		private LUCscSolver(
			IImplementationProvider provider, GlobalAlgebraicModel<CscMatrix> model)
			: base(model, "CSparseLUSolver")
		{
			this.provider = provider;
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
			CscMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution == null)
			{
				LinearSystem.Solution = Vector.CreateZero(systemSize);
			}
			//else linearSystem.Solution.Clear(); // no need to waste computational time on this in a direct solver

			// Factorization
			if (mustFactorize)
			{
				watch.Start();
				factorization = provider.CreateLUTriangulation();
				factorization.Factorize(matrix);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();
			factorization.SolveLinearSystem(LinearSystem.RhsVector, LinearSystem.Solution);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
		}

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
		{
			var watch = new Stopwatch();

			// Factorization
			CscMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			if (mustFactorize)
			{
				watch.Start();
				factorization = provider.CreateLUTriangulation();
				factorization.Factorize(matrix);
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
			private readonly IImplementationProvider provider;

			public Factory() 
			{
				this.provider = new ManagedSequentialImplementationProvider();

				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new AmdReordering(provider));
			}

			public IDofOrderer DofOrderer { get; set; }

			public LUCscSolver BuildSolver(GlobalAlgebraicModel<CscMatrix> model)
			{
				return new LUCscSolver(provider, model);
			}

			public GlobalAlgebraicModel<CscMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<CscMatrix>(model, DofOrderer, new CscMatrixAssembler(false, true));
		}
	}
}
