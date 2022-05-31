//using System;
//using System.Collections.Generic;
//using System.Text;
//using MGroup.LinearAlgebra.Matrices;
//using MGroup.MSolve.Discretization;
//using MGroup.MSolve.Solution;
//using MGroup.MSolve.Solution.Exceptions;
//using MGroup.MSolve.Solution.LinearSystem;

//namespace MGroup.Solvers.LinearSystem
//{
//	public class DistributedMatrix<TMatrix> : IGlobalMatrix
//		where TMatrix : IMatrix
//	{
//		private readonly Func<IGlobalVector, DistributedVector> checkCompatibleVector;
//		private readonly Func<IGlobalMatrix, DistributedMatrix<TMatrix>> checkCompatibleMatrix;

//		public DistributedMatrix(Guid format, Func<IGlobalVector, DistributedVector> checkCompatibleVector,
//			Func<IGlobalMatrix, DistributedMatrix<TMatrix>> checkCompatibleMatrix)
//		{
//			Format = format;
//			this.checkCompatibleVector = checkCompatibleVector;
//			this.checkCompatibleMatrix = checkCompatibleMatrix;
//		}

//		internal Guid Format { get; }

//		internal Dictionary<int, TMatrix> LocalMatrices { get; }

//		public void AxpyIntoThis(IGlobalMatrix otherMatrix, double otherCoefficient)
//		{
//			DistributedMatrix<TMatrix> distributedOther = checkCompatibleMatrix(otherMatrix);
//			foreach (int s in LocalMatrices.Keys)
//			{
//				TMatrix thisSubdomainMatrix = this.LocalMatrices[s];
//				TMatrix otherSubdomainMatrix = distributedOther.LocalMatrices[s];
//				thisSubdomainMatrix.AxpyIntoThis(otherSubdomainMatrix, otherCoefficient);
//			}
//		}

//		public void Clear()
//		{
//			foreach (int s in LocalMatrices.Keys)
//			{
//				this.LocalMatrices[s].Clear();
//			}
//		}

//		public IGlobalMatrix Copy()
//		{
//			var copy = new DistributedMatrix<TMatrix>(Format, checkCompatibleVector, checkCompatibleMatrix);
//			foreach (int s in this.LocalMatrices.Keys)
//			{
//				copy.LocalMatrices[s] = (TMatrix)this.LocalMatrices[s].Copy();
//			}
//			return copy;
//		}

//		public void LinearCombinationIntoThis(double thisCoefficient, IGlobalMatrix otherMatrix, double otherCoefficient)
//		{
//			DistributedMatrix<TMatrix> distributedOther = checkCompatibleMatrix(otherMatrix);
//			foreach (int s in LocalMatrices.Keys)
//			{
//				TMatrix thisSubdomainMatrix = this.LocalMatrices[s];
//				TMatrix otherSubdomainMatrix = distributedOther.LocalMatrices[s];
//				thisSubdomainMatrix.LinearCombinationIntoThis(thisCoefficient, otherSubdomainMatrix, otherCoefficient);
//			}
//		}

//		public void MultiplyVector(IGlobalVector input, IGlobalVector output)
//		{
//			DistributedVector distributedInput = checkCompatibleVector(input);
//			DistributedVector distributedOutput = checkCompatibleVector(output);
//			foreach (int s in LocalMatrices.Keys)
//			{
//				this.LocalMatrices[s].MultiplyIntoResult(distributedInput.LocalVectors[s], distributedOutput.LocalVectors[s]);
//			}
//			distributedOutput.SumOverlappingEntries();
//		}

//		public void ScaleIntoThis(double coefficient)
//		{
//			foreach (int s in LocalMatrices.Keys)
//			{
//				this.LocalMatrices[s].ScaleIntoThis(coefficient);
//			}
//		}
//	}
//}
