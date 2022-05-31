using System;
using System.Collections.Generic;
using System.Diagnostics;
using MGroup.LinearAlgebra.Matrices;
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
	/// Direct solver for models with only 1 subdomain. Uses Cholesky factorization on symmetric positive definite matrices
	/// stored in full format. Its purpose is mainly for testing, since it is inefficient for large linear systems resulting 
	/// from FEM .
	/// Authors: Serafeim Bakalakos
	/// </summary>
	public class DenseMatrixSolver : SingleSubdomainSolverBase<Matrix>
	{
		private readonly bool isMatrixPositiveDefinite; //TODO: actually there should be 3 states: posDef, symmIndef, unsymm

		private bool factorizeInPlace = true;
		private bool mustInvert = true;
		private Matrix inverse;

		private DenseMatrixSolver(GlobalAlgebraicModel<Matrix> model, bool isMatrixPositiveDefinite)
			: base(model, "DenseMatrixSolver")
		{
			this.isMatrixPositiveDefinite = isMatrixPositiveDefinite;
		}

		public override void HandleMatrixWillBeSet()
		{
			mustInvert = true;
			inverse = null;
		}

		public override void Initialize() { }

		public override void PreventFromOverwrittingSystemMatrices() => factorizeInPlace = false;

		/// <summary>
		/// Solves the linear system with back-forward substitution. If the matrix has been modified, it will be refactorized.
		/// </summary>
		public override void Solve()
		{
			var watch = new Stopwatch();
			if (LinearSystem.Solution.SingleVector == null)
			{
				LinearSystem.Solution.SingleVector = Vector.CreateZero(LinearSystem.Matrix.SingleMatrix.NumRows);
			}
			else LinearSystem.Solution.Clear(); // no need to waste computational time on this in a direct solver

			// Factorization
			if (mustInvert)
			{
				watch.Start();
				var matrix = LinearSystem.Matrix.SingleMatrix;
				if (isMatrixPositiveDefinite)
				{
					inverse = matrix.FactorCholesky(factorizeInPlace).Invert(true);
				}
				else
				{
					inverse = matrix.FactorLU(factorizeInPlace).Invert(true);
				}
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustInvert = false;
			}

			// Substitutions
			watch.Start();
			inverse.MultiplyIntoResult(LinearSystem.RhsVector.SingleVector, LinearSystem.Solution.SingleVector);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
		}

		protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
		{
			var watch = new Stopwatch();
			if (mustInvert)
			{
				watch.Start();
				var matrix = LinearSystem.Matrix.SingleMatrix;
				if (isMatrixPositiveDefinite)
				{
					inverse = matrix.FactorCholesky(factorizeInPlace).Invert(true);
				}
				else
				{
					inverse = matrix.FactorLU(factorizeInPlace).Invert(true);
				}
				watch.Stop();
				Logger.LogTaskDuration("Matrix factorization", watch.ElapsedMilliseconds);
				watch.Reset();
				mustInvert = false;
			}
			watch.Start();
			Matrix result = inverse.MultiplyRight(otherMatrix);
			watch.Stop();
			Logger.LogTaskDuration("Back/forward substitutions", watch.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
			return result;
		}

		public class Factory
		{
			public Factory() { }

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public bool IsMatrixPositiveDefinite { get; set; } = true;

			public DenseMatrixSolver BuildSolver(GlobalAlgebraicModel<Matrix> model)
				=> new DenseMatrixSolver(model, IsMatrixPositiveDefinite);

			public GlobalAlgebraicModel<Matrix> BuildAlgebraicModel(IModel model)
				=> new GlobalAlgebraicModel<Matrix>(model, DofOrderer, new DenseMatrixAssembler());
		}
	}
}
