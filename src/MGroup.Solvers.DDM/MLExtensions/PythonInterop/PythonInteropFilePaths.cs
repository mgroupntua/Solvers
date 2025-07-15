namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	public class PythonInteropFilePaths
	{
		private static Random rng = new Random();

		public PythonInteropFilePaths(string workDirectory)
		{
			var date = DateTime.Now;
			string pattern = "yyyy-MM-dd-hh-mm";
			//Guid guid = Guid.NewGuid();
			string guid = rng.Next().ToString("x");
			TempSubdirectoryNameOnly = $"_temp_{date.ToString(pattern)}_{guid}";
			TempSubdirectoryFullPath = $"{workDirectory}\\{TempSubdirectoryNameOnly}";
			//InputsArrays = TempSubdirectoryFullPath + "\\inputs_arrays.json";
			InputsSerialized = TempSubdirectoryFullPath + "\\inputs_serialized.json";
			//OutputsArrays = TempSubdirectoryFullPath + "\\outputs_arrays.json";
			OutputsSerialized = TempSubdirectoryFullPath + "\\outputs_serialized.json";
			LogPerformance = TempSubdirectoryFullPath + "\\log_performance.json";
			LogErrors = TempSubdirectoryFullPath + "\\log_errors.json";
		}

		//public string InputsArrays { get; }

		public string InputsSerialized { get; }

		public string LogErrors { get; }

		public string LogPerformance { get; }

		//public string OutputsArrays { get; }

		public string OutputsSerialized { get; }

		public string TempSubdirectoryFullPath { get; }

		public string TempSubdirectoryNameOnly { get; }

		public string MakePathForInputArray(string arrayName) => $"{TempSubdirectoryFullPath}\\input-{arrayName}.npy";

		public string MakePathForOutputArray(string arrayName) => $"{TempSubdirectoryFullPath}\\output-{arrayName}.npy";
	}
}
