using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution.Exceptions;
using MGroup.MSolve.Solution.LinearSystem;

//TODO: The vector types used for subdomain-level operations should implement IVectorAbstraction directly. 
//		This is basically an adapter and should be removed.
namespace MGroup.Solvers.LinearSystem
{
	public class GlobalVector : IGlobalVector
	{
		private readonly Func<IGlobalVector, GlobalVector> checkCompatibleVector;

		public GlobalVector(Guid format, Func<IGlobalVector, GlobalVector> checkCompatibleVector)
		{
			this.CheckForCompatibility = true;
			this.Format = format;
			this.checkCompatibleVector = checkCompatibleVector;
		}

		internal Guid Format { get; }

		public int Length => SingleVector.Length;

		//TODO: perhaps setting this should be done in the construction only
		//TODO: I would rather this was internal, but it is needed by the test classes
		public Vector SingleVector { get; set; }

		public bool CheckForCompatibility { get; set; }

		public void AxpyIntoThis(IGlobalVector otherVector, double otherCoefficient)
		{
			GlobalVector globalOtherVector = checkCompatibleVector(otherVector);
			this.SingleVector.AxpyIntoThis(globalOtherVector.SingleVector, otherCoefficient);
		}

		public void Clear() => SingleVector.Clear();

		public void CopyFrom(IGlobalVector other)
		{
			GlobalVector globalVector = checkCompatibleVector(other);
			this.SingleVector.CopyFrom(globalVector.SingleVector);
		}

		public double DotProduct(IGlobalVector otherVector)
		{
			GlobalVector globalVector = checkCompatibleVector(otherVector);
			return this.SingleVector.DotProduct(globalVector.SingleVector);
		}

		public IGlobalVector CreateZero()
		{
			var result = new GlobalVector(Format, checkCompatibleVector);
			result.SingleVector = Vector.CreateZero(SingleVector.Length);
			return result;
		}

		int IGlobalVector.Length() => SingleVector.Length;
		
		public void LinearCombinationIntoThis(double thisCoefficient, IGlobalVector otherVector, double otherCoefficient)
		{
			GlobalVector globalOtherVector = checkCompatibleVector(otherVector);
			this.SingleVector.LinearCombinationIntoThis(thisCoefficient, globalOtherVector.SingleVector, otherCoefficient);
		}

		public double Norm2() => this.SingleVector.Norm2();

		public void ScaleIntoThis(double coefficient) => this.SingleVector.ScaleIntoThis(coefficient);

		public void SetAll(double value)
		{
			this.SingleVector.SetAll(value);
		}
	}
}
