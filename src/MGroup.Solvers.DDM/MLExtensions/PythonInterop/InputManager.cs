//TODO: compress numpy arrays (possibly using only 1 file)
namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	using System;
	using System.Collections.Generic;
	using System.IO;
	using System.Linq;
	using System.Reflection;
	using System.Text;

	using MGroup.Solvers.DDM.MLExtensions.PythonInterop;

	using Newtonsoft.Json;

	using NumSharp;

	public class InputManager<T> : IoParamsManagerBase<T> where T : new()
    {
		public void WriteInputToFiles(T input, PythonInteropFilePaths paths)
		{
			// Write serializable properties to a json file 
			using (StreamWriter file = File.CreateText(paths.InputsSerialized))
			{
				serializer.Serialize(file, input);
			}

			// Write array properties to numpy files
			foreach (PropertyInfo prop in arrayProps)
			{
				var path = paths.MakePathForInputArray(prop.Name);
				var val = (Array)prop.GetValue(input);
				np.Save(val, path);
			}
		}
	}
}
