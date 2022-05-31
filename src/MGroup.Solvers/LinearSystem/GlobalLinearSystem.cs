using System;
using System.Collections.Generic;

using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Solution.LinearSystem;

//TODO: Perhaps the solvers should access directly the TMatrix Matrix, Vector Rhs, Vector Solution, instead of 
//		GlobalMatrix.SingleMatrix, GlobalVector.SingleVector, etc.
namespace MGroup.Solvers.LinearSystem
{
	public class GlobalLinearSystem<TMatrix> : IGlobalLinearSystem
		where TMatrix : class, IMatrix
	{
		private readonly Func<IGlobalVector, GlobalVector> checkCompatibleVector;
		private readonly Func<IGlobalMatrix, GlobalMatrix<TMatrix>> checkCompatibleMatrix;

		public GlobalLinearSystem(Func<IGlobalVector, GlobalVector> checkCompatibleVector,
			Func<IGlobalMatrix, GlobalMatrix<TMatrix>> checkCompatibleMatrix)
		{
			this.checkCompatibleVector = checkCompatibleVector;
			this.checkCompatibleMatrix = checkCompatibleMatrix;
			Observers = new HashSet<ILinearSystemObserver>();
		}

		IGlobalMatrix IGlobalLinearSystem.Matrix
		{
			get => Matrix;
			set
			{
				GlobalMatrix<TMatrix> globalMatrix = checkCompatibleMatrix(value);
				foreach (var observer in Observers)
				{
					observer.HandleMatrixWillBeSet();
				}
				Matrix = globalMatrix;
			}
		}

		//TODO: I would rather this was internal, but it is needed by the test classes
		public GlobalMatrix<TMatrix> Matrix { get; internal set; }


		public HashSet<ILinearSystemObserver> Observers { get; }

		IGlobalVector IGlobalLinearSystem.RhsVector
		{
			get => RhsVector;
			set
			{
				GlobalVector globalVector = checkCompatibleVector(value);
				RhsVector = globalVector;
			}
		}

		//TODO: I would rather this was internal, but it is needed by the test classes
		public GlobalVector RhsVector { get; internal set; }

		IGlobalVector IGlobalLinearSystem.Solution => Solution;

		//TODO: I would rather this was internal, but it is needed by the test classes
		public GlobalVector Solution { get; internal set; }
	}
}
