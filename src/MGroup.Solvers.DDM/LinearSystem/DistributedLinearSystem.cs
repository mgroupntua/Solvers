namespace MGroup.Solvers.DDM.LinearSystem
{
	using System;
	using System.Collections.Generic;

	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Solution.LinearSystem;

	public class DistributedLinearSystem<TMatrix> : IGlobalLinearSystem
		where TMatrix : class, IMatrix
	{
		private readonly Func<IVector, DistributedOverlappingVector> checkCompatibleVector;
		private readonly Func<IMatrix, DistributedOverlappingMatrix<TMatrix>> checkCompatibleMatrix;

		public DistributedLinearSystem(Func<IVector, DistributedOverlappingVector> checkCompatibleVector,
			Func<IMatrix, DistributedOverlappingMatrix<TMatrix>> checkCompatibleMatrix)
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
				DistributedOverlappingMatrix<TMatrix> globalMatrix = checkCompatibleMatrix(value);
				foreach (var observer in Observers)
				{
					observer.HandleMatrixWillBeSet();
				}

				Matrix = globalMatrix;
			}
		}

		public DistributedOverlappingMatrix<TMatrix> Matrix { get; set; }

		public HashSet<ILinearSystemObserver> Observers { get; }

		IVector IGlobalLinearSystem.RhsVector
		{
			get => RhsVector;
			set
			{
				DistributedOverlappingVector globalVector = checkCompatibleVector(value);
				RhsVector = globalVector;
			}
		}

		public DistributedOverlappingVector RhsVector { get; set; }

		IVector IGlobalLinearSystem.Solution
		{
			get => Solution;
			set
			{
				DistributedOverlappingVector globalVector = checkCompatibleVector(value);
				Solution = globalVector;
			}
		}

		public DistributedOverlappingVector Solution { get; set; }
	}
}
