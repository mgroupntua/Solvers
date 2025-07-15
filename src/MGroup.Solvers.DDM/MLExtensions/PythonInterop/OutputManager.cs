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
					//TODO: Also check that the file exists, before loading numpy arrays.
					throw new IOException($"Cannot read the array {prop.Name}. The file {path} does not contain a valid array");
				}
				prop.SetValue(output, loaded);
			}

			return output;
		}
	}
}
