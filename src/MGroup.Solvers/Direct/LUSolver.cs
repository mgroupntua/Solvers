using System;
using System.Diagnostics;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Triangulation;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.Solvers.LinearSystem;

namespace MGroup.Solvers.Direct
{
	/// <summary>
	/// Direct LU solver
	/// Authors: Papas Orestis, Christodoulou Theofilos
	/// </summary>
	public class LUSolver : SingleSubdomainSolverBase<CscMatrix>, IDisposable
	{
		private const bool useSuperNodalFactorization = true; // For faster back/forward substitutions.
		private readonly double factorizationPivotTolerance;

		private bool mustFactorize = true;
		private LUCSparseNet factorization;

		private LUSolver(GlobalAlgebraicModel<CscMatrix> model, double factorizationPivotTolerance) 
			: base(model, "LUSolver")
		{
			this.factorizationPivotTolerance = factorizationPivotTolerance;
		}

		~LUSolver()
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
				//factorization.Dispose(); //TODO: Maybe should open this again
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
			CscMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			if (LinearSystem.Solution.SingleVector == null)
			{
				LinearSystem.Solution.SingleVector = Vector.CreateZero(systemSize);
			}
			else LinearSystem.Solution.Clear();// no need to waste computational time on this in a direct solver

			// Factorization
			if (mustFactorize)
			{
				watch.Start();
				factorization = LUCSparseNet.Factorize(matrix);
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
			throw new NotImplementedException();
			//var watch = new Stopwatch();

			//// Factorization
			//CscMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			//int systemSize = matrix.NumRows;
			//if (mustFactorize)
			//{
			//	watch.Start();
			//	factorization = LUCSparseNet.Factorize(matrix);
			//	watch.Stop();
			//	Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
			//	watch.Reset();
			//	mustFactorize = false;
			//}

			//// Substitutions
			//watch.Start();
			//Matrix solutionVectors;
			//if (otherMatrix is Matrix otherDense)
			//{
			//	return factorization.SolveLinearSystems(otherDense.);
			//}
			//else
			//{
			//	try
			//	{
			//		// If there is enough memory, copy the RHS matrix to a dense one, to speed up computations. 
			//		//TODO: must be benchmarked, if it is actually more efficient than solving column by column.
			//		Matrix rhsVectors = otherMatrix.CopyToFullMatrix();
			//		solutionVectors = factorization.SolveLinearSystems(rhsVectors);
			//	}
			//	catch (InsufficientMemoryException) //TODO: what about OutOfMemoryException?
			//	{
			//		// Solution vectors
			//		int numRhs = otherMatrix.NumColumns;
			//		solutionVectors = Matrix.CreateZero(systemSize, numRhs);
			//		var solutionVector = Vector.CreateZero(systemSize);

			//		// Solve each linear system separately, to avoid copying the RHS matrix to a dense one.
			//		for (int j = 0; j < numRhs; ++j)
			//		{
			//			if (j != 0) solutionVector.Clear();
			//			Vector rhsVector = otherMatrix.GetColumn(j);
			//			factorization.SolveLinearSystem(rhsVector, solutionVector);
			//			solutionVectors.SetSubcolumn(j, solutionVector);
			//		}
			//	}
			//}
			//watch.Stop();
			//Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			//Logger.IncrementAnalysisStep();
			//return solutionVectors;
		}

		private void ReleaseResources()
		{
			if (factorization != null)
			{
				//factorization.Dispose(); //TODO: Maybe we should open this again
				factorization = null;
			}
		}

		public class Factory
		{
			public Factory() { }

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), AmdReordering.CreateWithSuiteSparseAmd());

			public double FactorizationPivotTolerance { get; set; } = 1E-15;

			public LUSolver BuildSolver(GlobalAlgebraicModel<CscMatrix> model)
				=> new LUSolver(model, FactorizationPivotTolerance);

			public GlobalAlgebraicModel<CscMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<CscMatrix>(model, DofOrderer, new CscMatrixAssembler(false));
		}
	}
}
