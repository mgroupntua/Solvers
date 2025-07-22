namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	using System;
	using System.Collections.Generic;
	using System.IO;
	using System.Linq;
	using System.Reflection;
	using System.Text;
	using DotNumerics.Optimization.TN;

	using MGroup.Solvers.DDM.MLExtensions.PythonInterop;

	using Newtonsoft.Json;

	using NumSharp;

	public class OutputManager<T> : IoParamsManagerBase<T> where T : new()
    {
		public T ReadOutputFromFiles(PythonInteropFilePaths paths)
		{
			T output = new();

			// Read serializable properties from a json file
			using (StreamReader file = File.OpenText(paths.OutputsSerialized))
			{
				object deserialized = serializer.Deserialize(file, paramsType);
				if (deserialized == null)
				{
					throw new IOException($"Could not read the non-array properties from the file");
				}
				output = (T)deserialized;
			}

			// Read array properties from numpy files
			foreach (PropertyInfo prop in arrayProps)
			{
				var path = paths.MakePathForOutputArray(prop.Name);
				Array loaded = np.LoadMatrix(path);
				if (loaded == null)
				{
					throw new IOException($"Cannot read the array {prop.Name}. The file {path} does not contain a valid array");
				}
				Array squeezed = ArrayConversions.SqueezeDimensions(loaded);
				prop.SetValue(output, squeezed);
			}

			return output;
		}

		private Array Squeeze(Array originalArray)
		{
			int rank = originalArray.Rank;
			var dimensions = new int[rank];
			var dimsToKeep = new List<int>(rank);
			for (int d = 0; d < rank; d++)
			{
				if (originalArray.GetLength(d) > 1)
				{
					dimsToKeep.Add(d);
				}
			}

			if (dimsToKeep.Count == rank)
			{
				return originalArray;
			}

			if (dimsToKeep.Count != 1)
			{
				throw new NotImplementedException();
			}

			int length = originalArray.Length;
			var result = Array.CreateInstance(originalArray.GetType().GetElementType(), length);
			for (int i = 0; i < length; i++)
			{
				result.SetValue(originalArray.GetValue(i), i);
			}
			//Array.Copy(originalArray, result, length); //this throws exception
			return result;
		}
	}
}
