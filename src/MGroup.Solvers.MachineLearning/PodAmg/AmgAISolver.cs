namespace MGroup.Solvers.MachineLearning
{
	using System;
	using System.Collections.Generic;

	using MGroup.LinearAlgebra.AlgebraicMultiGrid.PodAmg;
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MachineLearning.Utilities;
	using MGroup.MSolve.DataStructures;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;
	using MGroup.MSolve.Solution;
	using MGroup.Solvers.Logging;
	using MGroup.Solvers.LinearSystem;
	using MGroup.MSolve.Solution.LinearSystem;
	using System.Diagnostics;
	using MGroup.MachineLearning.TensorFlow;
	using MGroup.LinearAlgebra.AlgebraicMultiGrid;
	using MGroup.LinearAlgebra.Iterative.Stationary.CSR;
	using MGroup.LinearAlgebra.Extensions;

	public class AmgAISolver : ISolver
	{
		private const string name = "POD-AMG solver"; // for error messages

		private readonly IDofOrderer dofOrderer;
		private readonly PcgAlgorithm pcgAlgorithm;
		private readonly IPreconditioner initialPreconditioner;
		private readonly PodAmgPreconditioner amgPreconditioner;
		private readonly bool matrixPatternWillNotBeModified;
		private readonly int numSolutionVectorsForPod;
		private readonly int numPrincipalComponentsInPod;

		private double[] modelParametersCurrent;
		private CaeFffnSurrogate surrogate;
		private bool useAmgPreconditioner;

		private AmgAISolver(IDofOrderer dofOrderer, PcgAlgorithm pcgAlgorithm, bool matrixPatternWillNotBeModified,
			IPreconditioner initialPreconditioner, PodAmgPreconditioner amgPreconditioner,
			int numSolutionVectorsForPod, int numPrincipalComponentsInPod, CaeFffnSurrogate surrogate)
		{
			this.dofOrderer = dofOrderer;
			this.pcgAlgorithm = pcgAlgorithm;
			this.matrixPatternWillNotBeModified = matrixPatternWillNotBeModified;
			this.initialPreconditioner = initialPreconditioner;
			this.amgPreconditioner = amgPreconditioner;
			this.numSolutionVectorsForPod = numSolutionVectorsForPod;
			this.numPrincipalComponentsInPod = numPrincipalComponentsInPod;
			this.surrogate = surrogate;

			useAmgPreconditioner = false;
			PreviousSolutionVectors = new List<Vector>();
			PreviousModelParameters = new List<double[]>();
			Logger = new SolverLogger(name);
		}

		public GlobalAlgebraicModel<CsrMatrix> AlgebraicModel { get; private set; }

		IGlobalLinearSystem ISolver.LinearSystem => LinearSystem;

		public GlobalLinearSystem<CsrMatrix> LinearSystem { get; private set; }

		public ISolverLogger Logger { get; }

		public string Name => name;

		public List<Vector> PreviousSolutionVectors { get; }

		public List<double[]> PreviousModelParameters { get; }

		public void HandleMatrixWillBeSet() { }

		public void Initialize() { }

		public Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix) => throw new NotImplementedException();

		public void PreventFromOverwrittingSystemMatrices()
		{
			// No factorization is done.
		}

		public void SetModel(double[] modelParameters, IModel model)
		{
			modelParametersCurrent = modelParameters.Copy();
			AlgebraicModel = new GlobalAlgebraicModel<CsrMatrix>(model, dofOrderer, new CsrMatrixAssembler(true));
			this.LinearSystem = AlgebraicModel.LinearSystem;
			this.LinearSystem.Observers.Add(this);
		}

		public void Solve()
		{
			if (useAmgPreconditioner)
			{
				SolveUsingPodAmgPreconditioner();
			}
			else
			{
				if (PreviousSolutionVectors.Count < numSolutionVectorsForPod)
				{
					Vector solution = SolveUsingInitialPreconditioner();
					if (PreviousSolutionVectors.Count > 0)
					{
						if (solution.Length != PreviousSolutionVectors[0].Length)
						{
							throw new Exception("All solution vectors must have the same length, but the " +
								$"{PreviousSolutionVectors.Count + 1}th solution vector has length={solution.Length}, " +
								$"while the previous ones had length={PreviousSolutionVectors[0].Length}");
						}
					}
					PreviousSolutionVectors.Add(solution);
					PreviousModelParameters.Add(modelParametersCurrent);
				}
				else
				{
					useAmgPreconditioner = true;
					TrainBasedOnFirstSolutions();
					SolveUsingPodAmgPreconditioner();
				}
			}
		}

		private void TrainBasedOnFirstSolutions()
		{
			// Gather all previous solution vectors as columns of a matrix
			int numSamples = PreviousSolutionVectors.Count;
			int numDofs = PreviousSolutionVectors[0].Length;
			Matrix solutionVectors = Matrix.CreateZero(numDofs, numSamples);
			for (int j = 0; j < numSamples; ++j)
			{
				solutionVectors.SetSubcolumn(j, PreviousSolutionVectors[j]);
			}

			// Free up some memory by deleting the stored solution vectors
			PreviousSolutionVectors.Clear();

			// AMG-POD training
			amgPreconditioner.Initialize(solutionVectors, numPrincipalComponentsInPod);

			// Gather all previous model parameters
			if (PreviousModelParameters.Count != numSamples)
			{
				throw new Exception($"Have gathered {PreviousModelParameters.Count} sets of model parameters, " +
					$"but {numSamples} solution vectors, while using initial preconditioner.");
			}

			int numParameters = modelParametersCurrent.Length;
			var parametersAsArray = new double[numSamples, numParameters];
			for (int i = 0; i < numSamples; ++i)
			{
				if (PreviousModelParameters[i].Length != numParameters)
				{
					throw new Exception("The model parameter sets do not all have the same size");
				}

				for (int j = 0; j < numParameters; ++j)
				{
					parametersAsArray[i, j] = PreviousModelParameters[i][j];
				}
			}

			// CAE-FFNN training. Dimension 0 must be the number of samples.
			double[,] solutionsAsArray = solutionVectors.Transpose().CopytoArray2D();
			surrogate.TrainAndEvaluate(parametersAsArray, solutionsAsArray, null);
		}

		private Vector SolveUsingInitialPreconditioner()
		{
			IMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;

			// Preconditioning
			initialPreconditioner.UpdateMatrix(matrix, !matrixPatternWillNotBeModified);

			// Iterative algorithm
			IterativeStatistics stats = pcgAlgorithm.Solve(matrix, initialPreconditioner,
				LinearSystem.RhsVector, LinearSystem.Solution, true);
			if (!stats.HasConverged)
			{
				throw new IterativeSolverNotConvergedException(Name + " did not converge to a solution. PCG algorithm with "
					+ $"diagonal preconditioner run for {stats.NumIterationsRequired} iterations and the residual norm ratio was"
					+ $" {stats.ResidualNormRatioEstimation}");
			}

			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			return LinearSystem.Solution.Copy();
		}

		private void SolveUsingPodAmgPreconditioner()
		{
			CsrMatrix matrix = LinearSystem.Matrix;
			int systemSize = matrix.NumRows;
			Vector rhs = LinearSystem.RhsVector;

			// Use ML prediction as initial guess.
			double[] parameters = modelParametersCurrent.Copy();
			double[] prediction = surrogate.Predict(parameters);
			var solution = Vector.CreateFromArray(prediction);
			((IGlobalLinearSystem)LinearSystem).Solution = solution;

			amgPreconditioner.UpdateMatrix(matrix, !matrixPatternWillNotBeModified);

			IterativeStatistics stats = pcgAlgorithm.Solve(matrix, amgPreconditioner, rhs, solution, false);
			if (!stats.HasConverged)
			{
				throw new IterativeSolverNotConvergedException(Name + " did not converge to a solution. PCG algorithm with "
					+ $"AMG-POD preconditioner run for {stats.NumIterationsRequired} iterations and the residual norm ratio was"
					+ $" {stats.ResidualNormRatioEstimation}");
			}
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
		}

		public class Factory
		{
			private readonly int numSolutionVectorsForPod;
			private readonly int numPrincipalComponentsInPod;
			private readonly CaeFffnSurrogate.Builder surrogateBuilder;

			public Factory(int numSolutionVectorsForPod, int numPrincipalComponentsInPod,
				CaeFffnSurrogate.Builder surrogateBuilder)
			{
				this.numSolutionVectorsForPod = numSolutionVectorsForPod;
				this.numPrincipalComponentsInPod = numPrincipalComponentsInPod;
				this.surrogateBuilder = surrogateBuilder;
			}

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public bool MatrixPatternWillNotBeModified { get; set; } = false;

			public double PcgConvergenceTolerance { get; set; } = 1E-5;

			public IMaxIterationsProvider PcgMaxIterationsProvider { get; set; } = new PercentageMaxIterationsProvider(1.0);

			public AmgAISolver BuildSolver()
			{
				var pcgFactory = new PcgAlgorithm.Factory();
				pcgFactory.ResidualTolerance = PcgConvergenceTolerance;
				pcgFactory.MaxIterationsProvider = PcgMaxIterationsProvider;
				var pcgAlgorithm = pcgFactory.Build();

				var initialPreconditioner = new JacobiPreconditioner();

				var smoothing = new MultigridLevelSmoothing()
					.AddPreSmoother(new GaussSeidelIterationCsr(forwardDirection: true), 1)
					.AddPreSmoother(new GaussSeidelIterationCsr(forwardDirection: false), 1)
					.SetPostSmoothersSameAsPreSmoothers();
				var amgPreconditioner = new PodAmgPreconditioner(
					keepOnlyNonZeroPrincipalComponents: true, smoothing, numIterations: 1);

				return new AmgAISolver(DofOrderer, pcgAlgorithm, MatrixPatternWillNotBeModified, initialPreconditioner,
					amgPreconditioner, numSolutionVectorsForPod, numPrincipalComponentsInPod, surrogateBuilder.BuildSurrogate());
			}
		}
	}
}
