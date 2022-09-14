using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Solution.Exceptions;
using MGroup.MSolve.Solution.LinearSystem;

//TODO: The matrix types used for subdomain-level operations should implement IMatrixAbstraction directly. 
//		This is basically an adapter and should be removed.
//TODO: All the checks in this and other matrix and vector classes should be provided by the linear system object. Even better,
//		use an ILinearAlgebraModel component that has info about the dofs, subdomains, matrices and vectors. This will act as a 
//		mediator to let all of these work together with as much safety as possible and also create new zeroed out ones. It will 
//		also expose ILinearSystem, IGlobalMatrixAssembler, IGlobalVectorAssembler, etc, so that analyzers and providers can work 
//		with those directly.
namespace MGroup.Solvers.LinearSystem
{
	public class GlobalMatrix<TMatrix> : IGlobalMatrix
		where TMatrix : class, IMatrix
	{
		private readonly Func<IGlobalVector, GlobalVector> checkCompatibleVector;
		private readonly Func<IGlobalMatrix, GlobalMatrix<TMatrix>> checkCompatibleMatrix;

		public GlobalMatrix(Guid format, Func<IGlobalVector, GlobalVector> checkCompatibleVector, Func<IGlobalMatrix, GlobalMatrix<TMatrix>> checkCompatibleMatrix)
		{
			this.CheckForCompatibility = true;
			this.Format = format;
			this.checkCompatibleVector = checkCompatibleVector;
			this.checkCompatibleMatrix = checkCompatibleMatrix;
		}

		internal Guid Format { get; }

		public TMatrix SingleMatrix { get; set; }

		public bool CheckForCompatibility { get; set; }

		public void AxpyIntoThis(IGlobalMatrix otherMatrix, double otherCoefficient)
		{
			GlobalMatrix<TMatrix> otherGlobalMatrix = checkCompatibleMatrix(otherMatrix);
			this.SingleMatrix.AxpyIntoThis(otherGlobalMatrix.SingleMatrix, otherCoefficient);
		}

		public void Clear() => SingleMatrix.Clear();

		public IGlobalMatrix Copy()
		{
			var result = new GlobalMatrix<TMatrix>(Format, checkCompatibleVector, checkCompatibleMatrix);
			result.SingleMatrix = (TMatrix)SingleMatrix.Copy();
			return result;
		}

		public void LinearCombinationIntoThis(double thisCoefficient, IGlobalMatrix otherMatrix, double otherCoefficient)
		{
			GlobalMatrix<TMatrix> otherGlobalMatrix = checkCompatibleMatrix(otherMatrix);
			this.SingleMatrix.LinearCombinationIntoThis(thisCoefficient, otherGlobalMatrix.SingleMatrix, otherCoefficient);
		}

		public void MultiplyVector(IGlobalVector input, IGlobalVector output)
		{
			GlobalVector globalInput = checkCompatibleVector(input);
			GlobalVector globalOutput = checkCompatibleVector(output);
			this.SingleMatrix.MultiplyIntoResult(globalInput.SingleVector, globalOutput.SingleVector);
		}

		public void ScaleIntoThis(double coefficient) => this.SingleMatrix.ScaleIntoThis(coefficient);
	}
}
