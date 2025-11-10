using System;
using System.Collections.Generic;

using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution.LinearSystem;

//TODO: Perhaps the solvers should access directly the TMatrix Matrix, Vector Rhs, Vector Solution, instead of 
//		GlobalMatrix.SingleMatrix, GlobalVector.SingleVector, etc.
namespace MGroup.Solvers.LinearSystem
{
	public class GlobalLinearSystem<TMatrix> : IGlobalLinearSystem
		where TMatrix : class, IMatrix
	{
		private readonly Func<IVector, Vector> checkCompatibleVector;
		private readonly Func<IMatrix, TMatrix> checkCompatibleMatrix;

		public GlobalLinearSystem(Func<IVector, Vector> checkCompatibleVector,
			Func<IMatrix, TMatrix> checkCompatibleMatrix)
		{
			this.checkCompatibleVector = checkCompatibleVector;
			this.checkCompatibleMatrix = checkCompatibleMatrix;
			Observers = new HashSet<ILinearSystemObserver>();
		}

		IMatrix IGlobalLinearSystem.Matrix
		{
			get => Matrix;
			set
			{
				TMatrix matrix = checkCompatibleMatrix(value);
				foreach (var observer in Observers)
				{
					observer.HandleMatrixWillBeSet();
				}
				Matrix = matrix;
			}
		}

		//TODO: I would rather this was internal, but it is needed by the test classes
		public TMatrix Matrix { get; internal set; }

		public HashSet<ILinearSystemObserver> Observers { get; }

		IVector IGlobalLinearSystem.RhsVector
		{
			get => RhsVector;
			set
			{
				Vector vector = checkCompatibleVector(value);
				RhsVector = vector;
			}
		}

		//TODO: I would rather this was internal, but it is needed by the test classes
		public Vector RhsVector { get; internal set; }

		IVector IGlobalLinearSystem.Solution
		{
			get => Solution;
			set
			{
				Vector vector = checkCompatibleVector(value);
				Solution = vector;
			}
		}

		//TODO: I would rather this was internal, but it is needed by the test classes
		public Vector Solution { get; internal set; }
	}
}
