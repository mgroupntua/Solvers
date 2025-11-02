namespace MGroup.Solvers.DDM.Tests.FetiDP
{
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
	using MGroup.Solvers.DDM.FetiDP;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.InterfaceProblem;
	using MGroup.Solvers.DDM.FetiDP.Preconditioning;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.Tests.ExampleModels;
	using MGroup.Solvers.Results;
	using MGroup.Solvers.Tests;
	using MGroup.Solvers.Tests.TempUtilityClasses;
	using Xunit;

	[Collection("Sequential")]
	public static class SimpleFetiDPSolverTests
	{
		public static TheoryData<bool, bool, bool, IEnvironmentChoice, IImplementationProviderChoice> DataForBrick3DTest
		{
			get
			{
				var data = new TheoryData<bool, bool, bool, IEnvironmentChoice, IImplementationProviderChoice>();
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, false, false, false);
				return data;
			}
		}

		[Theory]
		[MemberData(nameof(DataForBrick3DTest))]
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
			var solverFactory = new FetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, laProviderForSolver, cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCsc.Factory());

			solverFactory.Preconditioner = new FetiDPDirichletPreconditioner();
			solverFactory.InterfaceProblemSolverFactory = new FetiDPInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10,
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
			FetiDPSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

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
			int pcgIterationsExpected = 30;
			double pcgResidualNormRatioExpected = 6E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.InRange(stats.ResidualNormRatioEstimation, 0, pcgResidualNormRatioExpected);
		}

		public static TheoryData<bool, bool, bool, IEnvironmentChoice, IImplementationProviderChoice> DataForPlane2DTest
		{
			get
			{
				var data = new TheoryData<bool, bool, bool, IEnvironmentChoice, IImplementationProviderChoice>();
				TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, false, false, false);
				//TODO: The following fail. Fix them.
				//TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, false, false);
				//TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, true, false);
				//TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, false, true);
				//TestSettings.CombineTheoryDataWithAllProvidersAndEnvironments(data, true, true, true);
				return data;
			}
		}

		[Theory]
		[MemberData(nameof(DataForPlane2DTest))]
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
			var solverFactory = new FetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, laProviderForSolver, cornerDofs, new FetiDPSubdomainMatrixManagerSymmetricCsc.Factory());

			solverFactory.Preconditioner = new FetiDPDirichletPreconditioner();
			solverFactory.InterfaceProblemSolverFactory = new FetiDPInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 200,
				ResidualTolerance = 1E-10,
			};

			if (isCoarseProblemDistributed)
			{
				var coarseProblemFactory = new FetiDPCoarseProblemDistributed.Factory();
				if (useReorthogonalizedPcg)
				{
					var coarseProblemPcgBuilder = new ReorthogonalizedPcg.Factory();
					coarseProblemPcgBuilder.DirectionVectorsRetention = new FixedDirectionVectorsRetention(30, true);
					coarseProblemPcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(200);
					coarseProblemPcgBuilder.ResidualTolerance = 2E-12;
					coarseProblemPcgBuilder.Convergence = new PureResidualConvergence();
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
			FetiDPSolver<SymmetricCscMatrix> solver = solverFactory.BuildSolver(model, algebraicModel);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel);
			var childAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(algebraicModel, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			// Check results
			NodalResults expectedResults = Plane2DExample.GetExpectedNodalValues(problem.ActiveDofs);
			double tolerance = 1E-7;
			environment.DoPerNode(subdomainID =>
			{
				NodalResults computedResults = algebraicModel.ExtractAllResults(subdomainID, solver.LinearSystem.Solution);
				Assert.True(expectedResults.IsSuperSetOf(computedResults, tolerance, out string msg), msg);
			});

			//Debug.WriteLine($"Num PCG iterations = {solver.PcgStats.NumIterationsRequired}," +
			//    $" final residual norm ratio = {solver.PcgStats.ResidualNormRatioEstimation}");

			// Check convergence
			int pcgIterationsExpected = 14;
			double pcgResidualNormRatioExpected = 4E-11;
			IterativeStatistics stats = solver.InterfaceProblemSolutionStats;
			Assert.Equal(pcgIterationsExpected, stats.NumIterationsRequired);
			Assert.InRange(stats.ResidualNormRatioEstimation, 0, pcgResidualNormRatioExpected);
		}
	}
}
