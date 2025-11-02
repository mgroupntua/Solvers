namespace MGroup.Solvers.DDM.Tests.PFetiDP
{
	using System.Diagnostics;

	using MGroup.Constitutive.Structural;
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient.Reorthogonalization;
	using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.PFetiDP;
	using MGroup.Solvers.DDM.Psm;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;
	using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
	using MGroup.Solvers.DDM.Tests.ExampleModels;
	using MGroup.Solvers.Results;
	using MGroup.Solvers.Tests;
	using MGroup.Solvers.Tests.TempUtilityClasses;
	using Xunit;

	[Collection("Sequential")]
	public static class SimplePFetiDPSolverTests
	{
		public static TheoryData<bool, bool, bool, IEnvironmentChoice, IImplementationProviderChoice> TestData
		{
			get
			{
				var data = new TheoryData<bool, bool, bool, IEnvironmentChoice, IImplementationProviderChoice>();
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, false, false, false);
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, false, false);
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, true, false);
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, false, true);
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, true, true);
				return data;
			}
		}

		[Theory]
		[MemberData(nameof(TestData))]
		public static void TestForBrick3D(bool coarseDistributed, bool coarseJacobi, bool coarseReortho,
			IEnvironmentChoice environment, IImplementationProviderChoice provider)
		{
			TestForBrick3DInternal(coarseDistributed, coarseJacobi, coarseReortho, environment.Activate(), provider.Activate());
		}

		internal static void TestForBrick3DInternal(bool isCoarseProblemDistributed, bool useCoarseJacobiPreconditioner,
			bool useReorthogonalizedPcg, IComputeEnvironment environment, IImplementationProvider laProviderForSolver)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Brick3DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Brick3DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = Brick3DExample.GetCornerDofs(model);

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, laProviderForSolver, new PsmSubdomainMatrixManagerSymmetricCsc.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCsc.Factory(true));

			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			if (isCoarseProblemDistributed)
			{
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				if (useReorthogonalizedPcg)
				{
					var coarseProblemPcgBuilder = new ReorthogonalizedPcg.Factory();
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
					coarseProblemPcgBuilder.ResidualTolerance = 2E-12;
					coarseProblemPcgBuilder.DirectionVectorsRetention = new FixedDirectionVectorsRetention(40, true);
					coarseProblemPcgBuilder.Convergence = new PureResidualConvergence();
					coarseProblemPcgBuilder.ThrowExceptionIfNotConvergence = true;
					coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				}
				else
				{
					var coarseProblemPcgBuilder = new PcgAlgorithm.Factory();
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
					coarseProblemPcgBuilder.ResidualTolerance = 2E-12;
					coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				}

				coarseProblemFactory.UseJacobiPreconditioner = useCoarseJacobiPreconditioner;
				solverFactory.CoarseProblemFactory = coarseProblemFactory;
			}
			else
			{
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCsc(laProviderForSolver);
				solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			}

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel);
			var childAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(algebraicModel, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			NodalResults expectedResults = Brick3DExample.GetExpectedNodalValues(problem.ActiveDofs);
			double tolerance = 1E-7;
			environment.DoPerNode(subdomainID =>
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomainID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 29;
			double pcgResidualNormRatioExpected = 6.641424316172292E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}

		[Theory]
		[MemberData(nameof(TestData))]
		public static void TestForPlane2D(bool coarseDistributed, bool coarseJacobi, bool coarseReortho,
			IEnvironmentChoice environment, IImplementationProviderChoice provider)
		{
			TestForPlane2DInternal(coarseDistributed, coarseJacobi, coarseReortho, environment.Activate(), provider.Activate());
		}

		internal static void TestForPlane2DInternal(bool isCoarseProblemDistributed, bool useCoarseJacobiPreconditioner,
			bool useReorthogonalizedPcg, IComputeEnvironment environment, IImplementationProvider laProviderForSolver)
		{
			// Environment
			ComputeNodeTopology nodeTopology = Plane2DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			// Model
			IModel model = Plane2DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = Plane2DExample.GetCornerDofs(model);

			// Solver
			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, laProviderForSolver, new PsmSubdomainMatrixManagerSymmetricCsc.Factory(),
				cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCsc.Factory(true));

			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10
			};

			if (isCoarseProblemDistributed)
			{
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				if (useReorthogonalizedPcg)
				{
					var coarseProblemPcgBuilder = new ReorthogonalizedPcg.Factory();
					coarseProblemPcgBuilder.DirectionVectorsRetention = new FixedDirectionVectorsRetention(30, true);
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
					coarseProblemPcgBuilder.ResidualTolerance = 1E-11;
					coarseProblemPcgBuilder.Convergence = new PureResidualConvergence();
					coarseProblemPcgBuilder.ThrowExceptionIfNotConvergence = true;
					coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				}
				else
				{
					var coarseProblemPcgBuilder = new PcgAlgorithm.Factory();
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
					coarseProblemPcgBuilder.ResidualTolerance = 2E-12;
					coarseProblemFactory.CoarseProblemSolver = coarseProblemPcgBuilder.Build();
				}

				coarseProblemFactory.UseJacobiPreconditioner = useCoarseJacobiPreconditioner;
				solverFactory.CoarseProblemFactory = coarseProblemFactory;
			}
			else
			{
				var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCsc(laProviderForSolver);
				solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			}

			DistributedAlgebraicModel<SymmetricCscMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			PsmSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel);
			var childAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(algebraicModel, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			NodalResults expectedResults = Plane2DExample.GetExpectedNodalValues(problem.ActiveDofs);
			double tolerance = 1E-6; // relaxed from 1E-7
			environment.DoPerNode(subdomainID =>
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomainID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int precision = 10;
			int pcgIterationsExpected = 14;
			double pcgResidualNormRatioExpected = 2.868430313362798E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.Equal(pcgResidualNormRatioExpected, stats.ResidualNormRatioEstimation, precision);
		}
	}
}
