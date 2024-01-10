namespace MGroup.Solvers.MachineLearning
{
	using System;
	using System.Collections.Generic;

	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.AlgebraicMultiGrid;
	using MGroup.LinearAlgebra.Iterative.AlgebraicMultiGrid.PodAmg;
	using MGroup.LinearAlgebra.Iterative.AlgebraicMultiGrid.Smoothing;
	using MGroup.LinearAlgebra.Iterative.GaussSeidel;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Constitutive.Structural.MachineLearning.Surrogates;
	using MGroup.MachineLearning.Utilities;
	using MGroup.MSolve.DataStructures;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers;
	using MGroup.Solvers.AlgebraicModel;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;
	using MGroup.MSolve.Solution;
	using MGroup.Solvers.Logging;
	using MGroup.Solvers.LinearSystem;
	using MGroup.MSolve.Solution.LinearSystem;
	using System.Diagnostics;

	public class AmgAISolver : ISolver
	{
		private const string name = "POD-AMG solver"; // for error messages

		private readonly IDofOrderer _dofOrderer;
		private readonly PcgAlgorithm _pcgAlgorithm;
		private readonly IPreconditionerFactory _initialPreconditionerFactory;
		private readonly PodAmgPreconditioner.Factory _amgPreconditionerFactory;
		private readonly int _numSolutionVectorsForPod;
		private readonly int _numPrincipalComponentsInPod;

		private double[] _modelParametersCurrent;
		private CaeFffnSurrogate _surrogate;
		private bool _useAmgPreconditioner;

		private AmgAISolver(IDofOrderer dofOrderer, PcgAlgorithm pcgAlgorithm,
			IPreconditionerFactory initialPreconditionerFactory, PodAmgPreconditioner.Factory amgPreconditionerFactory,
			int numSolutionVectorsForPod, int numPrincipalComponentsInPod, CaeFffnSurrogate surrogate)
		{
			_dofOrderer = dofOrderer;
			_pcgAlgorithm = pcgAlgorithm;
			_initialPreconditionerFactory = initialPreconditionerFactory;
			_amgPreconditionerFactory = amgPreconditionerFactory;
			_numSolutionVectorsForPod = numSolutionVectorsForPod;
			_numPrincipalComponentsInPod = numPrincipalComponentsInPod;
			_surrogate = surrogate;

			_useAmgPreconditioner = false;
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
			_modelParametersCurrent = modelParameters.Copy();
			AlgebraicModel = new GlobalAlgebraicModel<CsrMatrix>(model, _dofOrderer, new CsrMatrixAssembler(true));
			this.LinearSystem = AlgebraicModel.LinearSystem;
			this.LinearSystem.Observers.Add(this);
		}

		public void Solve()
		{
			if (_useAmgPreconditioner)
			{
				SolveUsingPodAmgPreconditioner();
			}
			else
			{
				if (PreviousSolutionVectors.Count < _numSolutionVectorsForPod)
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
					PreviousModelParameters.Add(_modelParametersCurrent);
				}
				else
				{
					_useAmgPreconditioner = true;
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
			_amgPreconditionerFactory.Initialize(solutionVectors, _numPrincipalComponentsInPod);

			// Gather all previous model parameters
			if (PreviousModelParameters.Count != numSamples)
			{
				throw new Exception($"Have gathered {PreviousModelParameters.Count} sets of model parameters, " +
					$"but {numSamples} solution vectors, while using initial preconditioner.");
			}

			int numParameters = _modelParametersCurrent.Length;
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
			_surrogate.TrainAndEvaluate(parametersAsArray, solutionsAsArray, null);
		}

		private Vector SolveUsingInitialPreconditioner()
		{
			IMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;

			// Preconditioning
			IPreconditioner preconditioner = _initialPreconditionerFactory.CreatePreconditionerFor(matrix);

			// Iterative algorithm
			IterativeStatistics stats = _pcgAlgorithm.Solve(matrix, preconditioner,
				LinearSystem.RhsVector.SingleVector, LinearSystem.Solution.SingleVector,
				true, () => Vector.CreateZero(systemSize));
			if (!stats.HasConverged)
			{
				throw new IterativeSolverNotConvergedException(Name + " did not converge to a solution. PCG algorithm with "
					+ $"diagonal preconditioner run for {stats.NumIterationsRequired} iterations and the residual norm ratio was"
					+ $" {stats.ResidualNormRatioEstimation}");
			}

			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			return LinearSystem.Solution.SingleVector.Copy();
		}

		private void SolveUsingPodAmgPreconditioner()
		{
			CsrMatrix matrix = LinearSystem.Matrix.SingleMatrix;
			int systemSize = matrix.NumRows;
			Vector rhs = LinearSystem.RhsVector.SingleVector;

			// Use ML prediction as initial guess.
			double[] parameters = _modelParametersCurrent.Copy();
			double[] prediction = _surrogate.Predict(parameters);
			var solution = Vector.CreateFromArray(prediction);
			LinearSystem.Solution.SingleVector = solution;

			var preconditioner =  _amgPreconditionerFactory.CreatePreconditionerFor(matrix);

			IterativeStatistics stats = _pcgAlgorithm.Solve(matrix, preconditioner, rhs, solution, 
				false, () => Vector.CreateZero(systemSize));
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
			private readonly int _numSolutionVectorsForPod;
			private readonly int _numPrincipalComponentsInPod;
			private readonly CaeFffnSurrogate.Builder _surrogateBuilder;

			public Factory(int numSolutionVectorsForPod, int numPrincipalComponentsInPod, 
				CaeFffnSurrogate.Builder surrogateBuilder)
			{
				_numSolutionVectorsForPod = numSolutionVectorsForPod;
				_numPrincipalComponentsInPod = numPrincipalComponentsInPod;
				_surrogateBuilder = surrogateBuilder;
			}

			public IDofOrderer DofOrderer { get; set; }
				= new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public double PcgConvergenceTolerance { get; set; } = 1E-5;

			public IMaxIterationsProvider PcgMaxIterationsProvider { get; set; } = new PercentageMaxIterationsProvider(1.0);

			public AmgAISolver BuildSolver()
			{
				var pcgBuilder = new PcgAlgorithm.Builder();
				pcgBuilder.ResidualTolerance = PcgConvergenceTolerance;
				pcgBuilder.MaxIterationsProvider = PcgMaxIterationsProvider;
				var pcgAlgorithm = pcgBuilder.Build();

				var initialPreconditionerFactory = new JacobiPreconditioner.Factory();

				var amgPrecondFactory = new PodAmgPreconditioner.Factory();
				amgPrecondFactory.NumIterations = 1;
				amgPrecondFactory.SmootherBuilder = new GaussSeidelSmoother.Builder(
					new GaussSeidelIterationCsrSerial.Builder(),
					GaussSeidelSweepDirection.Symmetric,
					numIterations: 1);
				amgPrecondFactory.KeepOnlyNonZeroPrincipalComponents = true;

				return new AmgAISolver(DofOrderer, pcgAlgorithm, initialPreconditionerFactory, amgPrecondFactory,
					_numSolutionVectorsForPod, _numPrincipalComponentsInPod, _surrogateBuilder.BuildSurrogate());
			}
		}
	}
}
