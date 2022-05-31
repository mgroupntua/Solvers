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
	/// Direct solver for models with only 1 subdomain. Uses Cholesky factorization on sparse symmetric positive definite 
	/// matrices stored in Skyline format.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	public class SkylineSolver : SingleSubdomainSolverBase<SkylineMatrix>
	{
		private readonly double factorizationPivotTolerance;

		private bool factorizeInPlace = true;
		private bool mustFactorize = true;
		private LdlSkyline factorizedMatrix;

		private SkylineSolver(GlobalAlgebraicModel<SkylineMatrix> model, double factorizationPivotTolerance) 
			: base(model, "SkylineSolver")
		{
			this.factorizationPivotTolerance = factorizationPivotTolerance;
		}

		public override void HandleMatrixWillBeSet()
		{
			mustFactorize = true;
			factorizedMatrix = null;
		}

		public override void Initialize() { }

		public override void PreventFromOverwrittingSystemMatrices() => factorizeInPlace = false;

		/// <summary>
		/// Solves the linear system with back-forward substitution. If the matrix has been modified, it will be refactorized.
		/// </summary>
		public override void Solve()
		{
			var watch = new Stopwatch();
			SkylineMatrix matrix = LinearSystem.Matrix.SingleMatrix;
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
				factorizedMatrix = matrix.FactorLdl(factorizeInPlace, factorizationPivotTolerance);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();
			factorizedMatrix.SolveLinearSystem(LinearSystem.RhsVector.SingleVector, LinearSystem.Solution.SingleVector);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
		}

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
		{
			var watch = new Stopwatch();

			// Factorization
			SkylineMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			if (mustFactorize)
			{
				watch.Start();
				factorizedMatrix = matrix.FactorLdl(factorizeInPlace, factorizationPivotTolerance);
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustFactorize = false;
			}

			// Substitutions
			watch.Start();
			int numRhs = otherMatrix.NumColumns;
			var solutionVectors = Matrix.CreateZero(systemSize, numRhs);
			if (otherMatrix is Matrix otherDense)
			{
				factorizedMatrix.SolveLinearSystems(otherDense, solutionVectors);
			}
			else
			{
				try
				{
					// If there is enough memory, copy the RHS matrix to a dense one, to speed up computations. 
					//TODO: must be benchmarked, if it is actually more efficient than solving column by column.
					Matrix rhsVectors = otherMatrix.CopyToFullMatrix();
					factorizedMatrix.SolveLinearSystems(rhsVectors, solutionVectors);
				}
				catch (InsufficientMemoryException) //TODO: what about OutOfMemoryException?
				{
					// Solve each linear system separately, to avoid copying the RHS matrix to a dense one.
					var solutionVector = Vector.CreateZero(systemSize);
					for (int j = 0; j < numRhs; ++j)
					{
						if (j != 0) solutionVector.Clear();
						Vector rhsVector = otherMatrix.GetColumn(j);
						factorizedMatrix.SolveLinearSystem(rhsVector, solutionVector);
						solutionVectors.SetSubcolumn(j, solutionVector);
					}
				}
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
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public double FactorizationPivotTolerance { get; set; } = 1E-15;

			public SkylineSolver BuildSolver(GlobalAlgebraicModel<SkylineMatrix> model)
			{
				return new SkylineSolver(model, FactorizationPivotTolerance);
			}

			public GlobalAlgebraicModel<SkylineMatrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<SkylineMatrix>(model, DofOrderer, new SkylineMatrixAssembler());
		}
	}
}
