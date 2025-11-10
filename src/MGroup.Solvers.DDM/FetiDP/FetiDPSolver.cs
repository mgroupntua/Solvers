namespace MGroup.Solvers.DDM.FetiDP
{
	using System.Collections.Concurrent;
	using System.Diagnostics;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Implementations.Managed;
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.Solution;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.InterfaceProblem;
	using MGroup.Solvers.DDM.FetiDP.Preconditioning;
	using MGroup.Solvers.DDM.FetiDP.Reanalysis;
	using MGroup.Solvers.DDM.FetiDP.Scaling;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.FetiDP.Vectors;
	using MGroup.Solvers.DDM.LagrangeMultipliers;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.Output;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.DofOrdering.Reordering;
	using MGroup.Solvers.Logging;

	public class FetiDPSolver<TMatrix> : ISolver
		where TMatrix : class, IMatrix
	{
		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		private readonly IFetiDPCoarseProblem coarseProblem;
		private readonly ICornerDofSelection cornerDofs;
		private readonly ICrossPointStrategy crossPointStrategy;
		private readonly IComputeEnvironment environment;
		private readonly IInitialSolutionGuessStrategy initialSolutionGuessStrategy;
		private readonly IFetiDPInterfaceProblemMatrix interfaceProblemMatrix;
		private readonly ISystemSolutionIterativeMethod interfaceProblemSolver;
		private readonly IFetiDPInterfaceProblemVectors interfaceProblemVectors;
		private readonly IModel model;
		private readonly IModifiedCornerDofs modifiedCornerDofs;
		private readonly string name;
		private readonly ObjectiveConvergenceCriterion<TMatrix> objectiveConvergenceCriterion;
		private readonly IFetiDPPreconditioner preconditioner;
		private readonly IImplementationProvider provider;
		private readonly FetiDPReanalysisOptions reanalysis;
		private readonly IFetiDPScaling scaling;
		private readonly FetiDPSolutionRecovery solutionRecovery;
		private readonly ConcurrentDictionary<int, FetiDPSubdomainDofs> subdomainDofs;
		private readonly ConcurrentDictionary<int, SubdomainLagranges> subdomainLagranges;
		private readonly ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager> subdomainMatrices;
		private readonly ISubdomainTopology subdomainTopology;
		private readonly ConcurrentDictionary<int, FetiDPSubdomainRhsVectors> subdomainVectors;
		private readonly bool directSolverIsParallel = false;

		private int analysisIteration;
		private DistributedOverlappingIndexer lagrangeVectorIndexer;

		private FetiDPSolver(IComputeEnvironment environment, IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel,
			IImplementationProvider provider, IFetiDPSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory,
			bool explicitSubdomainMatrices, IFetiDPPreconditioner preconditioner,
			IFetiDPInterfaceProblemSolverFactory interfaceProblemSolverFactory, ICornerDofSelection cornerDofs,
			IFetiDPCoarseProblemFactory coarseProblemFactory, ICrossPointStrategy crossPointStrategy,
			bool isHomogeneous, DdmLogger logger,
			FetiDPReanalysisOptions reanalysis, string name = "FETI-DP Solver")
		{
			this.name = name;
			this.environment = environment;
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.provider = provider;
			this.cornerDofs = cornerDofs;
			this.crossPointStrategy = crossPointStrategy;
			this.subdomainTopology = algebraicModel.SubdomainTopology;
			this.LinearSystem = algebraicModel.LinearSystem;
			this.preconditioner = preconditioner;
			this.reanalysis = reanalysis;

			this.subdomainDofs = new ConcurrentDictionary<int, FetiDPSubdomainDofs>();
			this.subdomainLagranges = new ConcurrentDictionary<int, SubdomainLagranges>();
			this.subdomainMatrices = new ConcurrentDictionary<int, IFetiDPSubdomainMatrixManager>();
			this.subdomainVectors = new ConcurrentDictionary<int, FetiDPSubdomainRhsVectors>();
			environment.DoPerNode(subdomainID =>
			{
				SubdomainLinearSystem<TMatrix> linearSystem = algebraicModel.SubdomainLinearSystems[subdomainID];
				var dofs = new FetiDPSubdomainDofs(model.GetSubdomain(subdomainID), linearSystem);
				var lagranges = new SubdomainLagranges(model, subdomainID, subdomainTopology, dofs, crossPointStrategy);
				IFetiDPSubdomainMatrixManager matrices = matrixManagerFactory.CreateMatrixManager(provider, linearSystem, dofs);
				var vectors = new FetiDPSubdomainRhsVectors(linearSystem, dofs, lagranges, matrices);

				subdomainDofs[subdomainID] = dofs;
				subdomainLagranges[subdomainID] = lagranges;
				subdomainMatrices[subdomainID] = matrices;
				subdomainVectors[subdomainID] = vectors;
			});

			this.coarseProblem = coarseProblemFactory.CreateCoarseProblem(environment, algebraicModel.SubdomainTopology,
				s => subdomainDofs[s], s => subdomainMatrices[s]);
			if (reanalysis.GlobalCoarseProblemDofs)
			{
				modifiedCornerDofs = new GeneralModifiedCornerDofs(environment, s => subdomainDofs[s]);
			}
			else
			{
				modifiedCornerDofs = new NullModifiedCornerDofs();
			}

			if (isHomogeneous)
			{
				this.scaling = new HomogeneousScaling(
					environment, model, s => subdomainDofs[s], crossPointStrategy, reanalysis);
			}
			else
			{
				throw new NotImplementedException();
			}

			if (explicitSubdomainMatrices)
			{
				throw new NotImplementedException();
			}
			else
			{
				this.interfaceProblemMatrix = new FetiDPInterfaceProblemMatrixImplicit(
					environment, coarseProblem, subdomainLagranges, subdomainMatrices);
			}

			if (reanalysis.RhsVectors)
			{
				this.interfaceProblemVectors = new FetiDPInterfaceProblemVectors(
					environment, coarseProblem, subdomainLagranges, subdomainMatrices, subdomainVectors);
			}
			else
			{
				this.interfaceProblemVectors = new FetiDPInterfaceProblemVectors(
					environment, coarseProblem, subdomainLagranges, subdomainMatrices, subdomainVectors);
			}

			if (reanalysis.PreviousSolution)
			{
				//TODO: Refactor this. There must be more than 2 choices. This one here is appropriate for XFEM, but not nonlinear problems.
				this.initialSolutionGuessStrategy =
					new SameSolutionAtCommonDofsGuess(environment, reanalysis, s => subdomainLagranges[s]);
			}
			else
			{
				this.initialSolutionGuessStrategy = new ZeroInitialSolutionGuess();
			}

			this.solutionRecovery = new FetiDPSolutionRecovery(environment, coarseProblem, scaling,
				s => subdomainDofs[s], s => subdomainLagranges[s], s => subdomainMatrices[s], s => subdomainVectors[s]);

			IPcgResidualConvergence convergenceCriterion;
			if (interfaceProblemSolverFactory.UseObjectiveConvergenceCriterion)
			{
				this.objectiveConvergenceCriterion = new ObjectiveConvergenceCriterion<TMatrix>(
					environment, algebraicModel, solutionRecovery);
				convergenceCriterion = this.objectiveConvergenceCriterion;
			}
			else
			{
				convergenceCriterion = new ApproximatePcgResidualConvergence<TMatrix>(algebraicModel);
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

		public virtual void PreventFromOverwrittingSystemMatrices() { }

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
			PrepareCoarseProblem();
			PreparePreconditioner();
			PrepareInterfaceProblem();
			SolveInterfaceProblem();
			RecoverSolution();


			watchTotal.Stop();
			Logger.LogTaskDuration("Solution", watchTotal.ElapsedMilliseconds);
			Logger.IncrementAnalysisStep();
			++analysisIteration;
		}

		private bool GuessInitialSolution()
		{
			bool guessIsZero;
			if (analysisIteration == 0)
			{
				(interfaceProblemVectors.InterfaceProblemSolution, guessIsZero) =
					initialSolutionGuessStrategy.GuessFirstSolution(lagrangeVectorIndexer);
			}
			else
			{
				DistributedOverlappingVector previousSolution = interfaceProblemVectors.InterfaceProblemSolution;
				(interfaceProblemVectors.InterfaceProblemSolution, guessIsZero) =
					initialSolutionGuessStrategy.GuessNextSolution(lagrangeVectorIndexer, previousSolution);
			}
			return guessIsZero;
		}

		private void PrepareCoarseProblem()
		{
			var watch = new Stopwatch();
			watch.Start();
			watch.Restart();
			// Setup optimizations if coarse dofs are the same as in previous analysis
			modifiedCornerDofs.Update(reanalysis.ModifiedSubdomains);

			// Prepare coarse problem
			coarseProblem.FindCoarseProblemDofs(LoggerDdm, modifiedCornerDofs);
			coarseProblem.PrepareMatricesForSolution();
			watch.Stop();
			Logger.LogTaskDuration("Prepare coarse problem", watch.ElapsedMilliseconds);
		}

		private void PrepareGlobal2SubdomainMappings()
		{
			bool isFirstAnalysis = analysisIteration == 0;
			var watch = new Stopwatch();
			watch.Start();
			if (true/*isFirstAnalysis || !reanalysis.InterfaceProblemIndexer*/)
			{
				this.lagrangeVectorIndexer = new DistributedOverlappingIndexer(environment);
				environment.DoPerNode(subdomainID =>
				{
					subdomainLagranges[subdomainID].FindCommonLagrangesWithNeighbors();
				});
				this.lagrangeVectorIndexer.Initialize(
					subdomainID => subdomainLagranges[subdomainID].InitializeDistributedVectorIndexer());
			}
			else
			{
			}

			// Calculating scaling coefficients
			scaling.CalcScalingMatrices();
			watch.Stop();
			Logger.LogTaskDuration("Define interface problem dofs", watch.ElapsedMilliseconds);

		}

		private void PrepareInterfaceProblem()
		{
			var watch = new Stopwatch();
			watch.Start();
			interfaceProblemMatrix.Calculate(lagrangeVectorIndexer);
			interfaceProblemVectors.CalcInterfaceRhsVector(lagrangeVectorIndexer);
			watch.Stop();
			Logger.LogTaskDuration("Prepare interface problem", watch.ElapsedMilliseconds);
		}

		private void PreparePreconditioner()
		{
			bool isFirstAnalysis = analysisIteration == 0;
			var watch = new Stopwatch();
			watch.Start();
			preconditioner.Initialize(
				environment, lagrangeVectorIndexer, s => subdomainLagranges[s], s => subdomainMatrices[s], scaling);
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					preconditioner.CalcSubdomainMatrices(subdomainID);
				}
			});
			watch.Stop();
			Logger.LogTaskDuration("Prepare preconditioner", watch.ElapsedMilliseconds);
		}

		private void PrepareSubdomainDofs()
		{
			bool isFirstAnalysis = analysisIteration == 0;
			var watch = new Stopwatch();
			watch.Start();
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainDofSubsets
					|| reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing corner, boundary-remainder & internal dofs of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner, boundary-remainder & internal dofs of subdomain {subdomainID}");
					#endregion
					subdomainDofs[subdomainID].SeparateAllFreeDofs(cornerDofs);
					subdomainMatrices[subdomainID].ReorderRemainderDofs();
					subdomainLagranges[subdomainID].DefineSubdomainLagrangeMultipliers();
					subdomainLagranges[subdomainID].CalcSignedBooleanMatrices();
				}
				else
				{
					Debug.Assert(!subdomainDofs[subdomainID].IsEmpty);
				}
			});
			watch.Stop();
			Logger.LogTaskDuration("Subdomain level dofs", watch.ElapsedMilliseconds);
		}

		private void PrepareSubdomainMatrices()
		{
			bool isFirstAnalysis = analysisIteration == 0;
			var watch = new Stopwatch();
			watch.Start();
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.SubdomainSubmatrices
					|| reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing corner, boundary-remainder & internal submatrices of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner, boundary-remainder & internal submatrices of subdomain {subdomainID}");
					#endregion
					subdomainMatrices[subdomainID].HandleDofsWereModified();
					subdomainMatrices[subdomainID].ExtractKrrKccKrc();
					//subdomainMatricesPsm[subdomainID].InvertKrr();
				}
				else
				{
					Debug.Assert(!subdomainMatrices[subdomainID].IsEmpty);
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
						subdomainMatrices[subdomainID].InvertKrr();
						subdomainMatrices[subdomainID].CalcSchurComplementOfRemainderDofs();
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
						subdomainMatrices[subdomainID].InvertKrr();
						subdomainMatrices[subdomainID].CalcSchurComplementOfRemainderDofs();
					}
				});
			}

			watch.Stop();
			Logger.LogTaskDuration("Subdomain level matrices", watch.ElapsedMilliseconds);
		}

		private void PrepareSubdomainVectors()
		{
			bool isFirstAnalysis = analysisIteration == 0;
			var watch = new Stopwatch();
			watch.Start();
			environment.DoPerNode(subdomainID =>
			{
				if (isFirstAnalysis || !reanalysis.RhsVectors
					|| reanalysis.ModifiedSubdomains.IsRhsModified(subdomainID))
				{
					#region log
					//Console.WriteLine($"Processing corner, boundary-remainder & internal subvectors of subdomain {subdomainID}");
					//Debug.WriteLine($"Processing corner, boundary-remainder & internal subvectors of subdomain {subdomainID}");
					#endregion
					subdomainVectors[subdomainID].ExtractRhsSubvectors(
						fb => scaling.ScaleSubdomainRhsVector(subdomainID, fb));
					subdomainVectors[subdomainID].CalcCondensedRhsVector();
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
			// Having found the lagrange multipliers, now calculate the solution in term of primal dofs
			var watch = new Stopwatch();
			watch.Start();
			solutionRecovery.CalcPrimalSolution(
				interfaceProblemVectors.InterfaceProblemSolution, algebraicModel.LinearSystem.Solution);
			watch.Stop();
			Logger.LogTaskDuration("Recover solution at all dofs", watch.ElapsedMilliseconds);
		}

		private void SolveInterfaceProblem()
		{
			var watch = new Stopwatch();
			watch.Start();
			bool initalGuessIsZero = GuessInitialSolution();

			// Solver the interface problem
			IterativeStatistics stats = interfaceProblemSolver.Solve(
				interfaceProblemMatrix, preconditioner, interfaceProblemVectors.InterfaceProblemRhs,
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

		protected void LogSizes(IterativeStatistics stats)
		{
			if (LoggerDdm != null)
			{
				LoggerDdm.LogSolverConvergenceData(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
				LoggerDdm.LogProblemSize(0, algebraicModel.FreeDofIndexer.NumGlobalIndices);
				LoggerDdm.LogProblemSize(1, lagrangeVectorIndexer.NumGlobalIndices);

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
					subdomainID => lagrangeVectorIndexer.CountCommonEntriesOfNodeWithNeighbors(subdomainID).local);
				int totalRemoteTransfers = environment.AllReduceSum(
					subdomainID => lagrangeVectorIndexer.CountCommonEntriesOfNodeWithNeighbors(subdomainID).remote);
				LoggerDdm.LogTransfers(totalLocalTransfers, totalRemoteTransfers);
			}
		}

		public class Factory
		{
			private readonly ICornerDofSelection cornerDofs;
			private readonly IComputeEnvironment environment;
			private readonly IImplementationProvider provider;

			public Factory(IComputeEnvironment environment, IImplementationProvider provider, ICornerDofSelection cornerDofs,
				IFetiDPSubdomainMatrixManagerFactory<TMatrix> matrixManagerFactory)
			{
				this.environment = environment;
				this.cornerDofs = cornerDofs;
				this.provider = provider;

				CrossPointStrategy = new FullyRedundantLagranges();
				DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
				EnableLogging = false;
				ExplicitSubdomainMatrices = false;
				InterfaceProblemSolverFactory = new FetiDPInterfaceProblemSolverFactoryPcg();
				IsHomogeneousProblem = true;
				FetiDPMatricesFactory = matrixManagerFactory;
				Preconditioner = new FetiDPDirichletPreconditioner();
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCsc(provider);
				this.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
				ReanalysisOptions = FetiDPReanalysisOptions.CreateWithAllDisabled();
				SubdomainTopology = new SubdomainTopologyGeneral();
			}

			public IFetiDPCoarseProblemFactory CoarseProblemFactory { get; set; }

			public ICrossPointStrategy CrossPointStrategy { get; set; }

			public IDofOrderer DofOrderer { get; set; }

			public bool EnableLogging { get; set; }

			public bool ExplicitSubdomainMatrices { get; set; }

			public IFetiDPInterfaceProblemSolverFactory InterfaceProblemSolverFactory { get; set; }

			public bool IsHomogeneousProblem { get; set; }

			public IFetiDPSubdomainMatrixManagerFactory<TMatrix> FetiDPMatricesFactory { get; }

			public IFetiDPPreconditioner Preconditioner { get; set; }

			public FetiDPReanalysisOptions ReanalysisOptions { get; set; }

			public ISubdomainTopology SubdomainTopology { get; set; }

			public DistributedAlgebraicModel<TMatrix> BuildAlgebraicModel(IModel model)
			{
				return new DistributedAlgebraicModel<TMatrix>(
					environment, model, DofOrderer, SubdomainTopology, FetiDPMatricesFactory.CreateAssembler(),
					ReanalysisOptions);
			}

			public virtual FetiDPSolver<TMatrix> BuildSolver(IModel model, DistributedAlgebraicModel<TMatrix> algebraicModel)
			{
				DdmLogger logger = EnableLogging ? new DdmLogger(environment, "FETI-DP Solver", model.NumSubdomains) : null;
				return new FetiDPSolver<TMatrix>(environment, model, algebraicModel, provider, FetiDPMatricesFactory,
					ExplicitSubdomainMatrices, Preconditioner, InterfaceProblemSolverFactory, cornerDofs, CoarseProblemFactory,
					CrossPointStrategy, IsHomogeneousProblem, logger, ReanalysisOptions);
			}
		}
	}
}
