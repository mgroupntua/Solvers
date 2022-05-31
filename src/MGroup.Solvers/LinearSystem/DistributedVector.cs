//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using MGroup.LinearAlgebra.Vectors;
//using MGroup.MSolve.Discretization;
//using MGroup.MSolve.Solution.LinearSystem;
//using MGroup.Solvers.DofOrdering;

//namespace MGroup.Solvers.LinearSystem
//{
//	public class DistributedVector : IGlobalVector
//	{
//		private readonly IGlobalFreeDofOrdering globalDofOrdering;
//		private readonly Func<IGlobalVector, DistributedVector> checkCompatibleVector;
//		private readonly IEnumerable<ISubdomain> subdomains;

//		public DistributedVector(Guid format, Func<IGlobalVector, DistributedVector> checkCompatibleVector,
//			IEnumerable<ISubdomain> subdomains, IGlobalFreeDofOrdering globalDofOrdering)
//		{
//			this.Format = format;
//			this.checkCompatibleVector = checkCompatibleVector;
//			this.globalDofOrdering = globalDofOrdering;
//			this.subdomains = subdomains;
//		}

//		internal Guid Format { get; }

//		public Dictionary<int, Vector> LocalVectors { get; } = new Dictionary<int, Vector>(); //TODO: use concrete Vector

//		public void AxpyIntoThis(IGlobalVector otherVector, double otherCoefficient)
//		{
//			DistributedVector distributedVector = checkCompatibleVector(otherVector);
//			foreach (int s in LocalVectors.Keys)
//			{
//				this.LocalVectors[s].AxpyIntoThis(distributedVector.LocalVectors[s], otherCoefficient);
//			}
//		}

//		public double DotProduct(IGlobalVector otherVector) => throw new NotImplementedException();
//		public int Length() => throw new NotImplementedException();

//		public void Clear()
//		{
//			foreach (int s in LocalVectors.Keys)
//			{
//				this.LocalVectors[s].Clear();
//			}
//		}

//		public void CopyFrom(IGlobalVector other)
//		{
//			DistributedVector distributedVector = checkCompatibleVector(other);
//			foreach (int s in LocalVectors.Keys)
//			{
//				this.LocalVectors[s].CopyFrom(distributedVector.LocalVectors[s]);
//			}
//		}

//		public IGlobalVector CreateZero()
//		{
//			var result = new DistributedVector(Format, checkCompatibleVector, subdomains, globalDofOrdering);
//			foreach (int s in LocalVectors.Keys)
//			{
//				result.LocalVectors[s] = Vector.CreateZero(this.LocalVectors[s].Length);
//			}
//			return result;
//		}

//		public void LinearCombinationIntoThis(double thisCoefficient, IGlobalVector otherVector, double otherCoefficient)
//		{
//			DistributedVector distributedVector = checkCompatibleVector(otherVector);
//			foreach (int s in LocalVectors.Keys)
//			{
//				this.LocalVectors[s].LinearCombinationIntoThis(thisCoefficient, distributedVector.LocalVectors[s], otherCoefficient);
//			}
//		}

//		public double Norm2()
//		{
//			if (LocalVectors.Count == 1)
//			{
//				return LocalVectors.First().Value.Norm2();
//			}

//			double sum = 0.0;

//			foreach (ISubdomain subdomain in subdomains)
//			{
//				int s = subdomain.ID;
//				foreach ((INode node, IDofType dof, int idx) in globalDofOrdering.SubdomainDofOrderings[s].FreeDofs)
//				{
//					sum += LocalVectors[s][idx] * LocalVectors[s][idx] / node.Subdomains.Count;
//				}
//			}

//			return Math.Sqrt(sum);
//		}

//		public void ScaleIntoThis(double coefficient)
//		{
//			foreach (int s in LocalVectors.Keys)
//			{
//				this.LocalVectors[s].ScaleIntoThis(coefficient);
//			}
//		}

//		public void SetAll(double value)
//		{
//			foreach(int s in LocalVectors.Keys)
//			{
//				this.LocalVectors[s].SetAll(value);
//			}
//		}

//		public void SumOverlappingEntries()
//		{
//			if (LocalVectors.Count == 1) return;

//			var global = Vector.CreateZero(globalDofOrdering.NumGlobalFreeDofs);
//			foreach (ISubdomain subdomain in subdomains)
//			{
//				globalDofOrdering.AddVectorSubdomainToGlobal(subdomain, LocalVectors[subdomain.ID], global);
//			}
//			foreach (ISubdomain subdomain in subdomains)
//			{
//				globalDofOrdering.ExtractVectorSubdomainFromGlobal(subdomain, global, LocalVectors[subdomain.ID]);
//			}
//		}

//	}
//}
