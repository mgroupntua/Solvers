namespace MGroup.Solvers.DDM.Psm
{
	using System.Collections.Concurrent;
	using System.Collections.Generic;
	using System.Diagnostics;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Implementations.Managed;
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.Solution;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.Output;
	using MGroup.Solvers.DDM.PSM.Dofs;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;
	using MGroup.Solvers.DDM.PSM.Preconditioning;
	using MGroup.Solvers.DDM.PSM.Reanalysis;
	using MGroup.Solvers.DDM.PSM.Scaling;
	using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
	using MGroup.Solvers.DDM.PSM.Vectors;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;
	using MGroup.Solvers.Logging;

	public class PsmSolver<TMatrix> : ISolver
		where TMatrix : class, IMatrix
	{
		private const bool cacheDistributedVectorBuffers = true;

		protected readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		protected readonly bool directSolverIsParallel = false;
		protected readonly IComputeEnvironment environment;
		protected readonly IInitialSolutionGuessStrategy initialSolutionGuessStrategy;
		protected readonly IPsmInterfaceProblemMatrix interfaceProblemMatrix;
		protected readonly ISystemSolutionIterativeMethod interfaceProblemSolver;
		protected readonly IPsmInterfaceProblemVectors interfaceProblemVectors;
		protected readonly IModel model;
		protected readonly string name;
		private readonly ObjectiveConvergenceCriterion<TMatrix> objectiveConvergenceCriterion;
		protected /*readonly*/ IPsmPreconditioner preconditioner; //TODO: Make this readonly as well.
		protected readonly IImplementationProvider provider;
		protected readonly PsmReanalysisOptions reanalysis;
		protected readonly IBoundaryDofScaling scaling;
		protected readonly ConcurrentDictionary<int, PsmSubdomainDofs> subdomainDofsPsm;
		protected readonly ConcurrentDictionary<int, IPsmSubdomainMatrixManager> subdomainMatricesPsm;
		protected readonly ISubdomainTopology subdomainTopology;
		protected readonly ConcurrentDictionary<int, PsmSubdomainVectors> subdomainVectors;

		protected int analysisIteration;
		protected DistributedOverlappingIndexer boundaryDofIndexer;

		protected PsmSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel,
			IImplementationProvider provider, IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory,
			bool explicitSubdomainMatrices, IPsmPreconditioner preconditioner,
			IPsmInterfaceProblemSolverFactory interfaceProblemSolverFactory, bool isHomogeneous, DdmLogger logger,
			PsmReanalysisOptions reanalysis, string name = "PSM Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.provider = provider;
			this.subdomainTopology = algebraicModel.SubdomainTopology;
			this.LinearSystem = algebraicModel.LinearSystem;
			this.preconditioner = preconditioner;
			this.reanalysis = reanalysis;

			this.subdomainDofsPsm = new ConcurrentDictionary<int, PsmSubdomainDofs>();
			this.subdomainMatricesPsm = new ConcurrentDictionary<int, IPsmSubdomainMatrixManager>();
			this.subdomainVectors = new ConcurrentDictionary<int, PsmSubdomainVectors>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofs = new PsmSubdomainDofs(model.GetSubdomain(subdomainID), linearSystem, false);
				IPsmSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(provider, linearSystem, dofs);
				var vectors = new PsmSubdomainVectors(linearSystem, dofs, matrices);

				subdomainDofsPsm[subdomainID] = dofs;
				subdomainMatricesPsm[subdomainID] = matrices;
				subdomainVectors[subdomainID] = vectors;
			});

			if (isHomogeneous)
			{
				this.scaling = new HomogeneousScaling(environment, s => subdomainDofsPsm[s], reanalysis);
			}
			else
			{
				this.scaling = new HeterogeneousScaling(environment, subdomainTopology,
					s => algebraicModel.SubdomainLinearSystems[s], s => subdomainDofsPsm[s]);
			}

			if (explicitSubdomainMatrices)
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixExplicit(
					environment, s => subdomainMatricesPsm[s], reanalysis);
			}
			else
			{
				this.interfaceProblemMatrix = new PsmInterfaceProblemMatrixImplicit(environment, 
					s => subdomainDofsPsm[s], s => subdomainMatricesPsm[s]);
			}

			if (reanalysis.RhsVectors)
			{
				this.interfaceProblemVectors = new PsmInterfaceProblemVectorsReanalysis(
					environment, subdomainVectors, reanalysis.ModifiedSubdomains);
			}
			else
			{
				this.interfaceProblemVectors = new PsmInterfaceProblemVectors(environment, subdomainVectors);
			}

			if (reanalysis.PreviousSolution)
			{
				//TODO: Refactor this. There must be more than 2 choices. This one here is appropriate for XFEM, but not nonlinear problems.
				this.initialSolutionGuessStrategy = 
					new SameSolutionAtCommonDofsGuess(environment, reanalysis, s => subdomainDofsPsm[s]);
			}
			else
			{
				this.initialSolutionGuessStrategy = new ZeroInitialSolutionGuess();
			}

			IPcgResidualConvergence convergenceCriterion;
			if (interfaceProblemSolverFactory.UseObjectiveConvergenceCriterion)
			{
				this.objectiveConvergenceCriterion = new ObjectiveConvergenceCriterion<TMatrix>(
					environment, algebraicModel, s => subdomainVectors[s]);
				convergenceCriterion = this.objectiveConvergenceCriterion;
			}
			else
			{
				convergenceCriterion = new DefaultPcgConvergence();
			}
			this.interfaceProblemSolver = interfaceProblemSolverFactory.BuildIterativeMethod(convergenceCriterion);

			Logger = new SolverLogger(name);
			LoggerDdm = logger;

			if (provider is ManagedSequentialImplementationProvider)
			{
				directSolverIsParallel = false;
			}
			else
			{
				directSolverIsParallel = true;
			}

			analysisIteration = 0;
		}

		public IterativeStatistics InterfaceProblemSolutionStats { get; private set; }

		public IGlobalLinearSystem LinearSystem { get; }

		public ISolverLogger Logger { get; }

		public DdmLogger LoggerDdm { get; }

		public string Name => name;

		public virtual void HandleMatrixWillBeSet()
		{
		}

		public void Initialize() { }

		public virtual void PreventFromOverwrittingSystemMatrices() {}

		public virtual void Solve() 
		{
			var watchTotal = new Stopwatch();
			watchTotal.Start();

			if (LoggerDdm != null)
			{
				LoggerDdm.IncrementAnalysisIteration();
			}

			PrepareSubdomainDofs();
			PrepareSubdomainMatrices();
			PrepareGlobal2SubdomainMappings();
			PrepareSubdomainVectors();
			PrepareInterfaceProblem();
			CalcPreconditioner();

			SolveInterfaceProblem();
			RecoverSolution();

			watchTotal.Stop();
			Logger.LogTaskDuration("Solution", watchTotal.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
			++analysisIteration;
		}

		protected virtual void CalcPreconditioner()
		{
			var watch = new Stopwatch();
			watch.Start();
			preconditioner.Calculate(environment, boundaryDofIndexer, interfaceProblemMatrix);
			watch.Stop();
			Logger.LogTaskDuration("Prepare preconditioner", watch.ElapsedMilliseconds);
		}

		protected bool GuessInitialSolution()
		{
			bool guessIsZero;
			if (analysisIteration == 0)
			{
				(interfaceProblemVectors.InterfaceProblemSolution, guessIsZero) = 
					initialSolutionGuessStrategy.GuessFirstSolution(boundaryDofIndexer);
			}
			else
			{
				DistributedOverlappingVector previousSolution = interfaceProblemVectors.InterfaceProblemSolution;
				(interfaceProblemVectors.InterfaceProblemSolution, guessIsZero) =
					initialSolutionGuessStrategy.GuessNextSolution(boundaryDofIndexer, previousSolution);
			}
			interfaceProblemVectors.InterfaceProblemSolution.CacheSendRecvBuffers = cacheDistributedVectorBuffers;
			return guessIsZero;
		}

		protected void LogSizes(IterativeStatistics stats)
		{
			if (LoggerDdm != null)
			{
				LoggerDdm.LogSolverConvergenceData(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
				LoggerDdm.LogProblemSize(0, algebraicModel.FreeDofIndexer.NumGlobalIndices);
				LoggerDdm.LogProblemSize(1, boundaryDofIndexer.NumGlobalIndices);

				Dictionary<int, int> subdomainProblemSize = environment.AllGather(
					subdomainID => algebraicModel.LinearSystem.RhsVector.LocalVectors[subdomainID].Length);
				if (subdomainProblemSize != null)
				{
					foreach (var pair in subdomainProblemSize)
					{
						LoggerDdm.LogSubdomainProblemSize(pair.Key, pair.Value);
					}
				}

				int totalLocalTransfers = environment.AllReduceSum(
					subdomainID => boundaryDofIndexer.CountCommonEntriesOfNodeWithNeighbors(subdomainID).local);
				int totalRemoteTransfers = environment.AllReduceSum(
					subdomainID => boundaryDofIndexer.CountCommonEntriesOfNodeWithNeighbors(subdomainID).remote);
				LoggerDdm.LogTransfers(totalLocalTransfers, totalRemoteTransfers);
			}
		}

		protected void SolveInterfaceProblem()
		{
			var watch = new Stopwatch();
			watch.Start();
			bool initalGuessIsZero = GuessInitialSolution();

			// Solver the interface problem
			IterativeStatistics stats = interfaceProblemSolver.Solve(
				interfaceProblemMatrix.Matrix, preconditioner.Preconditioner, interfaceProblemVectors.InterfaceProblemRhs,
				interfaceProblemVectors.InterfaceProblemSolution, initalGuessIsZero);
			InterfaceProblemSolutionStats = stats;
			watch.Stop();

			Debug.WriteLine("Iterations for boundary problem = " + stats.NumIterationsRequired);
			Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
			Logger.LogTaskDuration("Interface problem solution", watch.ElapsedMilliseconds);
			LogSizes(stats);

			if (objectiveConvergenceCriterion != null)
			{
				Logger.LogTaskDuration("Objective PCG criterion", objectiveConvergenceCriterion.EllapsedMilliseconds);
				objectiveConvergenceCriterion.EllapsedMilliseconds = 0;
			}
		}

		private void PrepareGlobal2SubdomainMappings()
		{
			var watch = new Stopwatch();
			watch.Start();
			bool isFirstAnalysis = analysisIteration == 0;
			if (isFirstAnalysis || !reanalysis.InterfaceProblemIndexer)
			{
				this.boundaryDofIndexer = subdomainTopology.CreateDistributedVectorIndexer(
					s => subdomainDofsPsm[s].DofOrderingBoundary);
			}
			else
			{
				this.boundaryDofIndexer = subdomainTopology.RecreateDistributedVectorIndexer(
					s => subdomainDofsPsm[s].DofOrderingBoundary, this.boundaryDofIndexer,
					s => reanalysis.ModifiedSubdomains.IsConnectivityModified(s));
			}

			// Calculating scaling coefficients
			scaling.CalcScalingMatrices(boundaryDofIndexer);

			watch.Stop();
			Logger.LogTaskDuration("Global-subdomain mappings", watch.ElapsedMilliseconds);
		}

		private void PrepareInterfaceProblem()
		{
			var watch = new Stopwatch();
			watch.Start();
			interfaceProblemMatrix.Calculate(boundaryDofIndexer);
			interfaceProblemVectors.CalcInterfaceRhsVector(boundaryDofIndexer);
			watch.Stop();
			Logger.LogTaskDuration("Prepare interface problem", watch.ElapsedMilliseconds);
		}

		private void PrepareSubdomainDofs()
		{
			var watch = new Stopwatch();
			watch.Start();
			bool isFirstAnalysis = analysisIteration == 0;
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainDofSubsets
					|| reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing boundary & internal dofs of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing boundary & internal dofs of subdomain {subdomainID}");
					#endregion
					subdomainDofsPsm[subdomainID].SeparateFreeDofsIntoBoundaryAndInternal();
					subdomainMatricesPsm[subdomainID].ReorderInternalDofs();
				}
				else
				{
					Debug.Assert(!subdomainDofsPsm[subdomainID].IsEmpty);
				}
			});
			watch.Stop();
			Logger.LogTaskDuration("Subdomain level dofs", watch.ElapsedMilliseconds);
		}

		private void PrepareSubdomainMatrices()
		{
			var watch = new Stopwatch();
			watch.Start();
			bool isFirstAnalysis = analysisIteration == 0;
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing boundary & internal submatrices of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing boundary & internal submatrices of subdomain {subdomainID}");
					#endregion
					subdomainMatricesPsm[subdomainID].HandleDofsWereModified();
					subdomainMatricesPsm[subdomainID].ExtractKiiKbbKib();
					//subdomainMatricesPsm[subdomainID].InvertKii();
				}
				else
				{
					Debug.Assert(!subdomainMatricesPsm[subdomainID].IsEmpty);
				}
			});

			//TODO: This should be done together with the extraction. However SuiteSparse already uses multiple threads and should
			//		not be parallelized at subdomain level too. Instead environment.DoPerNode should be able to run tasks serially by reading a flag.
			if (directSolverIsParallel)
			{
				environment.DoPerNodeSerially(subdomainID =>
				{
					if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
					{
						//Console.WriteLine($"Invert Kii for subdomain {subdomainID}");
						subdomainMatricesPsm[subdomainID].InvertKii();
					}
				});
			}
			else
			{
				environment.DoPerNode(subdomainID =>
				{
					if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
					{
						//Console.WriteLine($"Invert Kii for subdomain {subdomainID}");
						subdomainMatricesPsm[subdomainID].InvertKii();
					}
				});
			}

			watch.Stop();
			Logger.LogTaskDuration("Subdomain level matrices", watch.ElapsedMilliseconds);
		}

		private void PrepareSubdomainVectors()
		{
			var watch = new Stopwatch();
			watch.Start();
			bool isFirstAnalysis = analysisIteration == 0;
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.RhsVectors
					|| reanalysis.ModifiedSubdomains.IsRhsModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing boundary & internal subvectors of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing boundary & internal subvectors of subdomain {subdomainID}");
					#endregion
					subdomainVectors[subdomainID].ExtractBoundaryInternalRhsVectors(
						fb => scaling.ScaleBoundaryRhsVector(subdomainID, fb));
				}
				else
				{
					Debug.Assert(!subdomainVectors[subdomainID].IsEmpty);
				}
			});
			watch.Stop();
			Logger.LogTaskDuration("Subdomain level vectors", watch.ElapsedMilliseconds);
		}

		private void RecoverSolution()
		{
			var watch = new Stopwatch();
			watch.Start();
			environment.DoPerNode(subdomainID =>
			{
				Vector subdomainBoundarySolution = interfaceProblemVectors.InterfaceProblemSolution.LocalVectors[subdomainID];
				subdomainVectors[subdomainID].CalcStoreSubdomainFreeSolution(subdomainBoundarySolution);
			});
			watch.Stop();
			Logger.LogTaskDuration("Recover solution at all dofs", watch.ElapsedMilliseconds);
		}

		public class Factory
		{
			protected readonly IComputeEnvironment environment;
			protected readonly IImplementationProvider provider;

			public Factory(IComputeEnvironment environment, IImplementationProvider provider,
				IPsmSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
			{
				this.environment = environment;
				this.provider = provider;

				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
				EnableLogging = false;
				ExplicitSubdomainMatrices = false;
				InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg();
				IsHomogeneousProblem = true;
				PsmMatricesFactory = matrixManagerFactory; //new PsmSubdomainMatrixManagerSymmetricCSparse.Factory();
				Preconditioner = new PsmPreconditionerIdentity();
				ReanalysisOptions = PsmReanalysisOptions.CreateWithAllDisabled();
				SubdomainTopology = new SubdomainTopologyGeneral();
			}

			public IDofOrderer DofOrderer { get; set; }

			public bool EnableLogging { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IPsmInterfaceProblemSolverFactory InterfaceProblemSolverFactory { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IPsmSubdomainMatrixManagerFactory<TMatrix> PsmMatricesFactory { get; }

			public IPsmPreconditioner Preconditioner { get; set; }

			public PsmReanalysisOptions ReanalysisOptions { get; set; }

			public ISubdomainTopology SubdomainTopology { get; set; }

			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
			{
				return new DistributedAlgebraicModel<TMatrix>(
					environment, model, DofOrderer, SubdomainTopology, PsmMatricesFactory.CreateAssembler(),
					ReanalysisOptions);
			}

			public virtual PsmSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
			{
				DdmLogger logger = EnableLogging ? new DdmLogger(environment, "PSM Solver", model.NumSubdomains) : null;
				return new PsmSolver<TMatrix>(environment, model, algebraicModel, provider, PsmMatricesFactory,
					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolverFactory, IsHomogeneousProblem,
					logger, ReanalysisOptions);
			}
		}
	}
}
