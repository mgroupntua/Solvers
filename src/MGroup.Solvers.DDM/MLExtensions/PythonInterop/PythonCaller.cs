using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;


using Newtonsoft.Json;

namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
	public class PythonCaller<TInput, TOutput>
		where TInput : new()
		where TOutput : new()
	{
		private readonly string workDirectory;
		private readonly string pythonInterpreterPath;
		private readonly string pythonScriptPath;
		private readonly InputManager<TInput> inputManager = new InputManager<TInput>();
		private readonly OutputManager<TOutput> outputManager = new OutputManager<TOutput>();

		public PythonCaller(string workDirectory, string pythonInterpreterPath, string pythonScriptPath)
		{
			this.workDirectory = workDirectory;
			this.pythonInterpreterPath = pythonInterpreterPath;
			this.pythonScriptPath = pythonScriptPath;
		}

		/// <summary>
		/// True (default) to delete any files created by this class. False to retain the files for manual inspection.
		/// </summary>
		public bool CleanupIOFiles { get; set; } = true;

		/// <summary>
		/// Specifies the milliseconds to wait before aborting the call to a Python script. 
		/// Indefinite waiting if <see cref="TimeoutMilliseconds"/> &lt; 0 (default).
		/// </summary>
		public int TimeoutMilliseconds { get; set; } = -1;

		public (TOutput output, PythonInteropPerformance performance) CallPython(TInput input)
		{
			var watch = new Stopwatch();
			watch.Restart();
			var performance = new PythonInteropPerformance();

			DirectoryInfo tempDirectory = null;
			var paths = new PythonInteropFilePaths(workDirectory);
			string processArgs = $"{pythonScriptPath} {paths.TempSubdirectoryFullPath}";
			watch.Stop();
			performance.CommunicationDuration += watch.ElapsedMilliseconds;

			try
			{
				// Write input files to filesystem
				watch.Restart();
				tempDirectory = Directory.CreateDirectory(paths.TempSubdirectoryFullPath);
				inputManager.WriteInputToFiles(input, paths);
				WriteLogFiles(paths);
				watch.Stop();
				performance.CommunicationDuration += watch.ElapsedMilliseconds;

				// Call script
				watch.Restart();
				RunSystemProcess(processArgs, paths.LogErrors);
				PythonCodeDurationsDto pythonDurations = ReadPythonPerformance(paths);
				watch.Stop();
				performance.IncludePythonDurations(watch.ElapsedMilliseconds, pythonDurations.IO, pythonDurations.Setup);

				// Read output files from filesystem
				watch.Restart();
				TOutput output = outputManager.ReadOutputFromFiles(paths);
				watch.Stop();
				performance.CommunicationDuration += watch.ElapsedMilliseconds;

				return (output, performance);
			}
			finally
			{
				// Cleanup
				if (CleanupIOFiles)
				{
					if (tempDirectory != null)
					{
						tempDirectory.Delete(recursive: true);
					}
				}
			}
		}

		private PythonCodeDurationsDto ReadPythonPerformance(PythonInteropFilePaths paths)
		{
			var serializer = new JsonSerializer();
			using (StreamReader file = File.OpenText(paths.LogPerformance))
			{
				var pythonDurations = (PythonCodeDurationsDto)serializer.Deserialize(file, typeof(PythonCodeDurationsDto));
				return pythonDurations;
			}
		}

		private void RunSystemProcess(string processArgs, string errorsLogPath)
		{
			var startInfo = new ProcessStartInfo(pythonInterpreterPath);
			startInfo.FileName = pythonInterpreterPath;
			startInfo.Arguments = processArgs;
			startInfo.UseShellExecute = false;
			startInfo.RedirectStandardOutput = true;
			startInfo.RedirectStandardError = true;
			int exitCode = -1;
			using (var process = Process.Start(startInfo))
			{
				process.WaitForExit(TimeoutMilliseconds);
				exitCode = process.ExitCode;
			}

			if (exitCode != 0)
			{
				// TODO: Exit code = 1 was thrown when the command line arguments do not match (or the parameters passed to a python function)

				if (exitCode == 100)
				{
					// The error message of Python code is written in the errors file
					using (var reader = new StreamReader(errorsLogPath))
					{
						string pythonErrorMsg = reader.ReadToEnd();
						var csharpErrorMsg = new StringBuilder();
						csharpErrorMsg.AppendLine($"Python script terminated with errors:");
						csharpErrorMsg.AppendLine($"**** Start of Python error message ***");
						csharpErrorMsg.AppendLine(pythonErrorMsg);
						csharpErrorMsg.AppendLine($"**** End of Python error message ***");
						throw new Exception(csharpErrorMsg.ToString());
					}
				}
				else
				{
					throw new Exception(
						$"Python script exited with code {exitCode}, instead of 0 (successful) or 100 (handled error).");
				}
			}
		}

		//TODO: This is not necessary. The Python helper can write them, since it has all the data
		private void WriteLogFiles(PythonInteropFilePaths paths)
		{
			// Performance
			var serializer = new JsonSerializer();
			using (StreamWriter file = File.CreateText(paths.LogPerformance))
			{
				serializer.Serialize(file, new PythonCodeDurationsDto());
			}

			// Errors
			using (StreamWriter writer = File.CreateText(paths.LogErrors))
			{
				writer.Write("No errors yet");
			}
		}
	}
}
