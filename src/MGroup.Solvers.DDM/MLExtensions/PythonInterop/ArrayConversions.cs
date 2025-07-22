namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Text;
	using System.Threading.Tasks;

	public static class ArrayConversions
	{
		public static Array SqueezeDimensions(Array original)
		{
			if (original == null)
			{
				throw new NullReferenceException();
			}

			var dimensions = new int[original.Rank];
			var dimsToKeep = new List<int>(original.Rank);
			for (int d = 0; d < original.Rank; d++)
			{
				if (original.GetLength(d) > 1)
				{
					dimsToKeep.Add(d);
				}
			}

			if (dimsToKeep.Count == original.Rank)
			{
				return original;
			}

			if ((dimsToKeep.Count == 1) && (original.Rank == 2))
			{
				if (original is double[,])
				{
					return Squeeze2Dto1D((double[,])original);
				}
				else if (original is float[,])
				{
					return Squeeze2Dto1D((float[,])original);
				}
				else if (original is int[,])
				{
					return Squeeze2Dto1D((int[,])original);
				}
				else if (original is string[,])
				{
					return Squeeze2Dto1D((string[,])original);
				}
			}

			throw new NotImplementedException();
		}

		public static T[] Squeeze2Dto1D<T>(T[,] original)
		{
			if (original.GetLength(0) == 1) // 1xN
			{
				T[] result = new T[original.GetLength(1)];
				for (int i = 0; i < result.Length; i++)
				{
					result[i] = original[0, i];
				}

				return result;
			}
			else // Nx1
			{
				T[] result = new T[original.GetLength(0)];
				for (int i = 0; i < result.Length; i++)
				{
					result[i] = original[i, 0];
				}

				return result;
			}
		}
	}
}
