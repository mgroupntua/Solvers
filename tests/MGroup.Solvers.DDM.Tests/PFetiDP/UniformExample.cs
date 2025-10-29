namespace MGroup.Solvers.DDM.Tests.PFetiDP
{
	using MGroup.Constitutive.Structural;
	using MGroup.Constitutive.Structural.Continuum;
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Implementations.Managed;
	using MGroup.LinearAlgebra.Implementations.NativeWin64;
	using MGroup.LinearAlgebra.Implementations.NativeWin64.SuiteSparse;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Triangulation;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.Solution;
	using MGroup.MSolve.Solution.AlgebraicModel;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.PFetiDP;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;
	using MGroup.Solvers.DDM.PSM.StiffnessMatrices;
	using MGroup.Solvers.DDM.Tests.Commons;

	public class UniformExample
	{
		/// <summary>
		/// Requires Win64, Intel MKl and SuiteSparse.
		/// </summary>
		public static void Run()
		{
			var laProviderForSolver = new NativeWin64ImplementationProvider();

			(Model model, ComputeNodeTopology nodeTopology) = DescribeModel().BuildMultiSubdomainModel();
			(ISolver solver, IAlgebraicModel algebraicModel) = SetupSolver(laProviderForSolver, model, nodeTopology);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel);
			var childAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(algebraicModel, problem, childAnalyzer);
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			Console.WriteLine($"Num dofs = {solver.LinearSystem.Solution.Length}");
		}

		private static UniformDdmModelBuilder3D DescribeModel()
		{
			var builder = new UniformDdmModelBuilder3D();
			builder.MinCoords = new double[] { 0, 0, 0 };
			builder.MaxCoords = new double[] { 8, 4, 4 }; 
			builder.NumElementsTotal = new int[] { 110, 55, 55 };
			builder.NumSubdomains = new int[] { 22, 11, 11};
			builder.NumClusters = new int[] { 1, 1, 1};
			builder.MaterialHomogeneous = new ElasticMaterial3D(2E7, 0.3);
			builder.PrescribeDisplacement(UniformDdmModelBuilder3D.BoundaryRegion.MinX, StructuralDof.TranslationX, 0.0);
			builder.PrescribeDisplacement(UniformDdmModelBuilder3D.BoundaryRegion.MinX, StructuralDof.TranslationY, 0.0);
			builder.PrescribeDisplacement(UniformDdmModelBuilder3D.BoundaryRegion.MinX, StructuralDof.TranslationZ, 0.0);
			builder.DistributeLoadAtNodes(UniformDdmModelBuilder3D.BoundaryRegion.MaxX, StructuralDof.TranslationY, 10000.0);

			return builder;
		}

		private static (ISolver solver, IAlgebraicModel algebraicModel) SetupSolver(IImplementationProvider provider, 
			Model model, ComputeNodeTopology nodeTopology)
		{
			// Environment
			IComputeEnvironment environment = new TplSharedEnvironment(false);
			environment.Initialize(nodeTopology);

			// Corner dofs
			model.ConnectDataStructures(); //TODOMPI: this is also done in the analyzer
			ICornerDofSelection cornerDofs = UniformDdmModelBuilder3D.FindCornerDofs(model);

			// Solver settings
			var psmMatrices = new PsmSubdomainMatrixManagerSymmetricCsc.Factory();
			var fetiDPMatrices = new FetiDPSubdomainMatrixManagerSymmetricCsc.Factory(true);
			var coarseProblemMatrix = new FetiDPCoarseProblemMatrixSymmetricCsc(provider);

			var solverFactory = new PFetiDPSolver<SymmetricCscMatrix>.Factory(
				environment, provider, psmMatrices, cornerDofs, fetiDPMatrices);
			solverFactory.CoarseProblemFactory = new FetiDPCoarseProblemGlobal.Factory(coarseProblemMatrix);
			solverFactory.EnableLogging = true;
			solverFactory.ExplicitSubdomainMatrices = false;
			solverFactory.InterfaceProblemSolverFactory = new PsmInterfaceProblemSolverFactoryPcg()
			{
				MaxIterations = 100,
				ResidualTolerance = 1E-7,
				UseObjectiveConvergenceCriterion = false,
			};

			// Create solver
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(model, algebraicModel);
			return (solver, algebraicModel);
		}
	}
}
