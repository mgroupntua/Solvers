using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;

using MGroup.MSolve.Solution;

using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Logging;
using MGroup.Solvers.DofOrdering;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.Solvers.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.AlgebraicModel;

namespace MGroup.Solvers
{
	/// <summary>
	/// Base implementation for solver that do not use domain decomposition.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	/// <typeparam name="TMatrix">The type of the linear system's matrix.</typeparam>
	public abstract class SingleSubdomainSolverBase<TMatrix> : ISolver
		where TMatrix : class, IMatrix
	{
		protected readonly GlobalAlgebraicModel<TMatrix> model;
		protected readonly string name; // for error messages

		protected SingleSubdomainSolverBase(GlobalAlgebraicModel<TMatrix> model, string name)
		{
			this.model = model;

			this.LinearSystem = model.LinearSystem;
			LinearSystem.Observers.Add(this);

			this.Logger = new SolverLogger(name);
		}

		public ISolverLogger Logger { get; }

		public string Name { get; }

		IGlobalLinearSystem ISolver.LinearSystem => LinearSystem;

		public GlobalLinearSystem<TMatrix> LinearSystem { get; set; }

		/// <summary>
		/// Solves multiple linear systems A * X = B, where: A is one of the matrices stored in <see cref="Solvers.LinearSystem"/>,
		/// B is the corresponding matrix in <paramref name="otherMatrix"/> and X is the corresponding matrix that will be 
		/// calculated as the result of inv(A) * B. 
		/// </summary>
		/// <param name="otherMatrix">
		/// The right hand side matrix for each subdomain. If the linear systems are A * X = B, then B is one of the matrices in
		/// <paramref name="otherMatrix"/>.</param>
		public Dictionary<int, Matrix> InverseSystemMatrixTimesOtherMatrix(Dictionary<int, IMatrixView> otherMatrix) //TODO: Remove this or make it subdomain agnostic
		{
			if (otherMatrix.Count != 1) throw new InvalidSolverException("There can only be 1 subdomain when using this solver");
			KeyValuePair<int, IMatrixView> idMatrixPair = otherMatrix.First();
			int id = idMatrixPair.Key;
			Debug.Assert(id == model.SubdomainID,
				"The matrix that will be multiplied with the inverse system matrix belongs to a different subdomain.");
			Matrix result = InverseSystemMatrixTimesOtherMatrix(idMatrixPair.Value);
			return new Dictionary<int, Matrix>() { { id, result } };
		}

		public abstract void Initialize();
		public abstract void HandleMatrixWillBeSet();
		public abstract void PreventFromOverwrittingSystemMatrices();
		public abstract void Solve();
		protected abstract Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix);
	}
}
