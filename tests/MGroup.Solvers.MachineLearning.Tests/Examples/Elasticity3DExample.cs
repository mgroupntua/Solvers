namespace MGroup.Solvers.MachineLearning.Tests.Examples
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Reflection.PortableExecutable;

	using global::MGroup.Solvers.MachineLearning.PodAmg;

	using MathNet.Numerics.Distributions;

	using MGroup.Solvers.MachineLearning.Tests.Mesh;
	using MGroup.Constitutive.Structural;
	using MGroup.Constitutive.Structural.BoundaryConditions;
	using MGroup.Constitutive.Structural.Continuum;
	using MGroup.Constitutive.Structural.MachineLearning.Surrogates;
	using MGroup.FEM.Structural.Continuum;
	using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
	using MGroup.LinearAlgebra.Output;
	using MGroup.LinearAlgebra.Output.Formatting;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MachineLearning.Utilities;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.Discretization.Meshes.Generation;
	using MGroup.MSolve.Solution;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.Direct;
	using System.IO;
	using System.Diagnostics;

	public static class Elasticity3DExample
	{
		private const int seed = 13;
		private const double cubeSide = 1.0;
		private const int numElementsPerSide = 16; // must be even
		private const string solutionVectorsFile = @"C:\Users\???\Desktop\AISolve\Example_2\solutionVectors.txt";
		private const string responsesFile = @"C:\Users\???\Desktop\AISolve\Example_2\responses.txt";
		private const bool writeSolutionsToFile = false;
		private const bool readSolutionsFromFileInsteadOfDiagonalPrecond = false;

		public static void Run()
		{
			int numAnalysesTotal = 300;
			int numSolutionsForTraining = 50; // paper used 300

			(double[] paramsE, double[] paramsP) = GenerateSamples(numAnalysesTotal);
			AmgAISolver solver = DefineSolver(numSolutionsForTraining);

			var responses = new List<double>(numAnalysesTotal);
			for (int i = 1; i <= numAnalysesTotal; i++)
			{
				Debug.WriteLine($"*************** Analysis {i}/{numAnalysesTotal} ***************");

				if (readSolutionsFromFileInsteadOfDiagonalPrecond && (i <= numSolutionsForTraining))
				{
					(double response, Vector solution) = ReadAnalysisResultsFromFile(i, solver);
				}
				else
				{
					(double response, Vector solution, int numPcgIterations) =
						RunSingleAnalysis(i, paramsE[i-1], paramsP[i-1], solver);
					responses.Add(response);
					Debug.WriteLine($"Number of PCG iterations = {numPcgIterations}. Dofs = {solution.Length}.");
					if (writeSolutionsToFile)
					{
						WriteSolutionToFile(solver.LinearSystem.Solution.SingleVector, response, i == 1);
					}
				}
			}

			double mean = responses.Average();
			Debug.WriteLine($"Total analyses: {numAnalysesTotal}. Training analyses: {numSolutionsForTraining}. " +
				$"Mean uTop={mean}");
		}

		private static (double response, Vector solution, int numPcgIterations) RunSingleAnalysis(int analysisNo,
			double E, double p, AmgAISolver solver)
		{
			Model model = CreateModel(E, p);
			Node monitorNode = FindMonitoredNode(model);

			solver.SetModel(new double[] { E, p }, model);
			var problem = new ProblemStructural(model, solver.AlgebraicModel);

			var linearAnalyzer = new LinearAnalyzer(solver.AlgebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(solver.AlgebraicModel, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			double computedDisplacement = solver.AlgebraicModel.ExtractSingleValue(
				solver.LinearSystem.Solution, monitorNode, StructuralDof.TranslationZ);
			int numIterations = solver.Logger.GetNumIterationsOfIterativeAlgorithm(analysisNo - 1);
			return (computedDisplacement, solver.LinearSystem.Solution.SingleVector, numIterations);
		}

		private static (double response, Vector solution) ReadAnalysisResultsFromFile(int analysisID, AmgAISolver solver)
		{
			string responseLine = ReadSpecificLineFromFile(responsesFile, analysisID);
			double response = double.Parse(responseLine);

			string solutionLine = ReadSpecificLineFromFile(solutionVectorsFile, analysisID);
			string[] words = solutionLine.Split(' ', StringSplitOptions.RemoveEmptyEntries);
			var solution = Vector.CreateZero(words.Length);
			for (int i = 0; i < words.Length; i++)
			{
				solution[i] = double.Parse(words[i]);
			}

			solver.PreviousSolutionVectors.Add(solution);
			return (response, solution);
		}

		private static string ReadSpecificLineFromFile(string path, int lineNumber)
		{
			using (var reader = new StreamReader(path))
			{
				for (int i = 1; i < lineNumber; i++)
				{
					reader.ReadLine();
				}

				return reader.ReadLine();
			}
		}

		private static Model CreateModel(double E, double p)
		{
			// Mesh
			double minX = 0, minY = 0, minZ = 0;
			double maxX = cubeSide, maxY = cubeSide, maxZ = cubeSide;
			var meshGenerator = new UniformMeshGenerator3D<Node>(minX, minY, minZ, maxX, maxY, maxZ,
				numElementsPerSide, numElementsPerSide, numElementsPerSide);
			(IReadOnlyList<Node> nodes, IReadOnlyList<CellConnectivity<Node>> elements) 
				= meshGenerator.CreateMesh((id, x, y, z) => new Node(id, x, y, z));

			// Create model
			var model = new Model();
			int s = 0;
			model.SubdomainsDictionary[s] = new Subdomain(s);

			// Materials
			double v = 0.3;
			var material = new ElasticMaterial3D(E, v);

			// Nodes
			foreach (Node node in nodes)
			{
				model.NodesDictionary[node.ID] = node;
			}

			// Elements
			var elementFactory = new ContinuumElement3DFactory(material, null);
			for (int e = 0; e < elements.Count; e++)
			{
				ContinuumElement3D element = elementFactory.CreateElement(elements[e].CellType, elements[e].Vertices);
				element.ID = e;
				model.ElementsDictionary[element.ID] = element;
				model.SubdomainsDictionary[s].Elements.Add(element);
			}

			// Constraints
			var constraints = new List<INodalDisplacementBoundaryCondition>();
			List<Node> constrainedNodes = FindNodesWith(IsNodeConstrained, model);
			foreach (Node node in constrainedNodes)
			{
				constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationX, amount: 0));
				constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationY, amount: 0));
				constraints.Add(new NodalDisplacement(node, StructuralDof.TranslationZ, amount: 0));
			}

			// Loads
			var loads = new List<INodalLoadBoundaryCondition>();
			List<Node> loadedNodes = FindNodesWith(IsNodeLoaded, model);
			double totalLoad = p / (cubeSide * cubeSide / 4);
			double loadPerNode = totalLoad / loadedNodes.Count;
			foreach (Node node in loadedNodes)
			{
				loads.Add(new NodalLoad(node, StructuralDof.TranslationZ, loadPerNode));
			}
			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

			return model;
		}

		private static Node FindMonitoredNode(Model model)
		{
			if (numElementsPerSide % 2 == 1)
			{
				throw new Exception($"The number of elements along each side of the domain is {numElementsPerSide}," +
					$" but it must be even, in order to define a central 'monitor' node");
			}

			var result = new List<Node>();
			foreach (Node node in model.NodesDictionary.Values)
			{
				double dx = cubeSide / numElementsPerSide;
				double tol = dx / 4.0;
				bool isMonitored = Math.Abs(node.Z - 1.0) < tol;
				isMonitored &= Math.Abs(node.X - cubeSide / 2.0) < tol;
				isMonitored &= Math.Abs(node.Y - cubeSide / 2.0) < tol;
				if (isMonitored)
				{
					result.Add(node);
				}
			}

			if (result.Count != 1)
			{
				throw new Exception($"Found {result.Count} monitor nodes, but only 1 was expected");
			}
			else
			{
				return result[0];
			}
		}

		private static List<Node> FindNodesWith(Func<Node, bool> predicate, Model model)
		{
			var result = new List<Node>();
			foreach (Node node in model.NodesDictionary.Values)
			{
				if (predicate(node))
				{
					result.Add(node);
				}
			}
			return result;
		}

		private static bool IsNodeConstrained(Node node)
		{
			double dx = cubeSide / numElementsPerSide;
			double tol = dx / 4.0;
			return Math.Abs(node.Z) < tol;
		}

		private static bool IsNodeLoaded(Node node)
		{
			double dx = cubeSide / numElementsPerSide;
			double tol = dx / 4.0;
			bool isLoaded = Math.Abs(node.Z - 1.0) < tol;
			isLoaded &= node.X > cubeSide / 4.0 - tol;
			isLoaded &= node.X < 3.0 / 4.0 * cubeSide + tol;
			isLoaded &= node.Y > cubeSide / 4.0 - tol;
			isLoaded &= node.Y < 3.0 / 4.0 * cubeSide + tol;
			return isLoaded;
		}

		private static (double[] paramsE, double[] paramsP) GenerateSamples(int numAnalysesTotal)
		{
			double meanE = 2000; //MPa
			double stdevE = 600; //MPa
			double meanP = -10; //MPa
			double stdevP = 3; //MPa
			var rng = new Random(seed);
			double[] paramsE = GenerateSamplesNormal(numAnalysesTotal, meanE, stdevE, rng).ToArray();
			double[] paramsP = GenerateSamplesNormal(numAnalysesTotal, meanP, stdevP, rng).ToArray();

			return (paramsE, paramsP);
		}

		private static IEnumerable<double> GenerateSamplesNormal(int count, double mean, double stddev, Random rng)
		{
			double[] samples = new double[count];
			Normal.Samples(rng, samples, mean, stddev);
			return samples;
			//return Normal.Samples(rng, mean, stddev);
		}

		private static IEnumerable<double> GenerateSamplesLogNormal(int count, double mean, double stddev, Random rng)
		{
			throw new NotImplementedException();
		}

		private static AmgAISolver DefineSolver(int numSolutionsForTraining)
		{
			var datasetSplitter = new DatasetSplitter();
			datasetSplitter.MinTestSetPercentage = 0.2;
			datasetSplitter.MinValidationSetPercentage = 0.0;
			datasetSplitter.SetOrderToContiguous(DataSubsetType.Training, DataSubsetType.Test);

			var surrogateBuilder = new CaeFffnSurrogate.Builder();
			surrogateBuilder.Splitter = datasetSplitter;
			surrogateBuilder.EncoderFilters = new int[] { 256, 128, 64, 32 };
			surrogateBuilder.DecoderFiltersWithoutOutput = new int[] { 64, 128, 256 };
			surrogateBuilder.CaeNumEpochs = 50; //paper used 500
			surrogateBuilder.CaeBatchSize = 20;
			surrogateBuilder.CaeLearningRate = 0.001f;
			surrogateBuilder.FfnnNumHiddenLayers = 6;
			surrogateBuilder.FfnnHiddenLayerSize = 64;
			surrogateBuilder.FfnnNumEpochs = 300; //paper used 3000
			surrogateBuilder.FfnnBatchSize = 20;
			surrogateBuilder.FfnnLearningRate = 0.0001f;

			int numPrincipalComponents = 8;
			var solverFactory = new AmgAISolver.Factory(numSolutionsForTraining, numPrincipalComponents, surrogateBuilder);
			solverFactory.PcgConvergenceTolerance = 1E-6;
			solverFactory.PcgMaxIterationsProvider = new PercentageMaxIterationsProvider(1.0);
			AmgAISolver solver = solverFactory.BuildSolver();

			return solver;
		}

		private static void WriteSolutionToFile(IVectorView solutionVector, double response, bool isFirstSolution)
		{
			double[] u = solutionVector.CopyToArray();
			var vectorWriter = new Array1DWriter();
			vectorWriter.ArrayFormat = Array1DFormat.PlainHorizontal;
			vectorWriter.WriteToFile(u, solutionVectorsFile, !isFirstSolution);

			using (var writer = new StreamWriter(responsesFile, !isFirstSolution))
			{
				writer.WriteLine(response);
			}
		}
	}
}
