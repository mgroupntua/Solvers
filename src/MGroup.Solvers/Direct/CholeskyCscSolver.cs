namespace MGroup.Solvers.Direct
{
	using System;
	using System.Diagnostics;

	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Implementations.Managed;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Triangulation;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;
	using MGroup.Solvers.LinearSystem;

	/// <summary>
	/// Direct solver for models with only 1 subdomain. Uses Cholesky factorization on sparse symmetric positive definite 
	/// matrices stored in symmetric (only the upper triangle) format. The default behaviour is to apply AMD reordering for the 
	/// model's freedom degrees. The factorization algorithm and AMD reordering may be implemented in native dlls. In that case,  
	/// the factorized matrix and other data stored in unmanaged memory will be properly disposed by this class.
	/// </summary>
	public class CholeskyCscSolver : SingleSubdomainSolverBase<SymmetricCscMatrix>, IDisposable
	{
		private readonly IImplementationProvider provider;

		private bool mustFactorize = true;
		private ICholeskySymmetricCsc factorization;

		private CholeskyCscSolver(IImplementationProvider provider, GlobalAlgebraicModel<SymmetricCscMatrix> model) 
			: base(model, "SkylineSolver")
		{
			this.provider = provider;
		}

		~CholeskyCscSolver()
		{
			ReleaseResources();
		}

		public void Dispose()
		{
			ReleaseResources();
			GC.SuppressFinalize(this);
		}

		public override void HandleMatrixWillBeSet()
		{
			mustFactorize = true;
			if (factorization != null)
			{
				factorization.Dispose();
				factorization = null;
			}
			//TODO: make sure the native memory allocated has been cleared. We need all the available memory we can get.
		}

		public override void Initialize() { }

		public override void PreventFromOverwrittingSystemMatrices()
		{
			// The factorization is done over different memory.
		}

		/// <summary>
		/// Solves the linear system with back-forward substitution. If the matrix has been modified, it will be refactorized.
		/// </summary>
		public override void Solve()
		{
			var watch = new Stopwatch();
			SymmetricCscMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution == null)
			{
				LinearSystem.Solution = Vector.CreateZero(systemSize);
			}
			else LinearSystem.Solution.Clear();// no need to waste computational time on this in a direct solver

			// Factorization
			if (mustFactorize)
			{
				watch.Start();
				factorization = provider.CreateCholeskyTriangulation();
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

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IReadOnlyMatrix otherMatrix)
		{
			var watch = new Stopwatch();

			// Factorization
			SymmetricCscMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			if (mustFactorize)
			{
				watch.Start();
				factorization = provider.CreateCholeskyTriangulation();
				factorization.Factorize(matrix);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();
			Matrix solutionVectors;
			if (otherMatrix is Matrix otherDense) return factorization.SolveLinearSystems(otherDense);
			else
			{
				try
				{
					// If there is enough memory, copy the RHS matrix to a dense one, to speed up computations. 
					//TODO: must be benchmarked, if it is actually more efficient than solving column by column.
					Matrix rhsVectors = otherMatrix.CopyToFullMatrix();
					solutionVectors = factorization.SolveLinearSystems(rhsVectors);
				}
				catch (InsufficientMemoryException) //TODO: what about OutOfMemoryException?
				{
					// Solution vectors
					int numRhs = otherMatrix.NumColumns;
					solutionVectors = Matrix.CreateZero(systemSize, numRhs);
					var solutionVector = Vector.CreateZero(systemSize);

					// Solve each linear system separately, to avoid copying the RHS matrix to a dense one.
					for (int j = 0; j < numRhs; ++j)
					{
						if (j != 0) solutionVector.Clear();
						Vector rhsVector = otherMatrix.GetColumn(j);
						factorization.SolveLinearSystem(rhsVector, solutionVector);
						solutionVectors.SetSubcolumn(j, solutionVector);
					}
				}
			}
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
			return solutionVectors;
		}

		private void ReleaseResources()
		{
			if (factorization != null)
			{
				factorization.Dispose();
				factorization = null;
			}
		}

		public class Factory
		{
			private readonly IImplementationProvider provider;

			public Factory(IImplementationProvider provider) 
			{
				this.provider = provider;
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new AmdReordering(provider));
			}

			public IDofOrderer DofOrderer { get; set; }

			//public double FactorizationPivotTolerance { get; set; } = 1E-15; // Not used by the libraries undfortunately

			public CholeskyCscSolver BuildSolver(GlobalAlgebraicModel<SymmetricCscMatrix> model)
				=> new CholeskyCscSolver(provider, model);

			public GlobalAlgebraicModel<SymmetricCscMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<SymmetricCscMatrix>(model, DofOrderer, new SymmetricCscMatrixAssembler());
		}
	}
}
