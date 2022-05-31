using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.DataStructures;

namespace MGroup.Solvers.Results
{
	public class NodalResults
	{
		public NodalResults(Table<int, int, double> data)
		{
			Data = data;
		}

		public Table<int, int, double> Data { get; }

		public bool IsSuperSetOf(NodalResults other, double tolerance, out string msg)
		{
			var comparer = new ValueComparer(tolerance);
			foreach ((int node, int dof, double otherValue) in other.Data)
			{
				bool thisValueExists = this.Data.TryGetValue(node, dof, out double thisValue);
				if (!thisValueExists)
				{
					msg = $"Node {node} dof {dof} does not exist in the superset.";
					return false;
				}

				if (!comparer.AreEqual(thisValue, otherValue))
				{
					msg = $"Node {node} dof {dof}: superset value = {thisValue}, subset value = {otherValue}";
					return false;
				}
			}

			msg = string.Empty;
			return true;
		}

		public NodalResults LinearCombination(double thisCoeff, NodalResults other, double otherCoeff)
		{
			var result = new NodalResults(new Table<int, int, double>());
			foreach ((int node, int dof, double thisValue) in this.Data)
			{
				double otherValue = other.Data[node, dof];
				double resultValue = thisCoeff * thisValue + otherCoeff * otherValue;
				result.Data[node, dof] = resultValue;
			}
			return result;
		}

		public double Norm2()
		{
			double sum = 0.0;
			foreach ((int node, int dof, double val) in this.Data)
			{
				sum += val * val;
			}
			return Math.Sqrt(sum);
		}

		public NodalResults Subtract(NodalResults other) => LinearCombination(1.0, other, -1.0);

		public void UnionWith(NodalResults other, double differentValueTolerance)
		{
			var comparer = new ValueComparer(differentValueTolerance);
			foreach ((int node, int dof, double otherValue) in other.Data)
			{
				bool thisValueExists = this.Data.TryGetValue(node, dof, out double thisValue);
				if (thisValueExists)
				{
					if (comparer.AreEqual(thisValue, otherValue))
					{
						this.Data[node, dof] = 0.5 * (thisValue + otherValue);
					}
					else
					{
						throw new ArgumentException($"Node {node} dof {dof}: the values of the 2 collections are too different");
					}
				}
				else
				{
					this.Data[node, dof] = otherValue;
				}
			}
		}
	}
}
