// Subdomains:
// /|
// /||-------|-------|-------|-------|
// /||  (4)  |  (5)  |  (6)  |  (7)  |
// /||   E1  |   E0  |   E0  |   E0  |
// /||-------|-------|-------|-------|
// /||  (0)  |  (1)  |  (2)  |  (3)  |
// /||   E1  |   E0  |   E0  |   E0  |
// /||-------|-------|-------|-------|
// /|
namespace MGroup.Solvers.DDM.Tests.ExampleModels
{
	using MGroup.Constitutive.Structural;
	using MGroup.Constitutive.Structural.Planar;
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.NumericalAnalyzers;
	using MGroup.Solvers.AlgebraicModel;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.Tests.Commons;
	using MGroup.Solvers.Direct;
	using MGroup.Solvers.Results;

	/// <summary>
	/// See Papagiannakis Bachelor thesis, pages 111-132
	/// </summary>
	public static class PapagiannakisExample_8modified
	{
		private const double E0 = 2.1E7, v = 0.3, thickness = 0.3;
		private const double load = 100;

		public static double[] MinCoords => new double[] { 0, 0 };

		//public static double[] MaxCoords => new double[] { 1.5, 0.75 };
		public static double[] MaxCoords => new double[] { 2.25, 0.75 };

		//public static int[] NumElements => new int[] { 6, 3 };
		public static int[] NumElements => new int[] { 9, 3 };

		//public static int[] NumSubdomains => new int[] { 2, 1 };
		public static int[] NumSubdomains => new int[] { 3, 1 };

		public static int[] NumClusters => new int[] { 1, 1 };

		public static int NumTotalDofs => 80;

		public static Model CreateSingleSubdomainModel(double stiffnessRatio)
		{
			UniformDdmModelBuilder2D builder = CreateDefaultModelBuilder(stiffnessRatio);
			builder.NumSubdomains = new int[] { 1, 1 };
			builder.NumClusters = new int[] { 1, 1 };
			return builder.BuildSingleSubdomainModel();
		}

		public static (Model model, ComputeNodeTopology topology) CreateMultiSubdomainModel(double stiffnessRatio)
		{
			UniformDdmModelBuilder2D builder = CreateDefaultModelBuilder(stiffnessRatio);
			builder.NumSubdomains = NumSubdomains;
			builder.NumClusters = NumClusters;
			return builder.BuildMultiSubdomainModel();
		}

		public static ICornerDofSelection GetCornerDofs(IModel model) => UniformDdmModelBuilder2D.FindCornerDofs(model, 2);

		public static NodalResults SolveWithSkylineSolver(Model model)
		{
			model.ConnectDataStructures();

			// Solver
			var solverFactory = new LdlSkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = solverFactory.BuildAlgebraicModel(model);
			LdlSkylineSolver solver = solverFactory.BuildSolver(algebraicModel);

			// Linear static analysis
			var problem = new ProblemStructural(model, algebraicModel);
			var childAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var parentAnalyzer = new StaticAnalyzer(algebraicModel, problem, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			NodalResults nodalDisplacements = algebraicModel.ExtractAllResults(solver.LinearSystem.Solution);
			return nodalDisplacements;
		}

		private static UniformDdmModelBuilder2D CreateDefaultModelBuilder(double stiffnessRatio)
		{
			var builder = new UniformDdmModelBuilder2D();
			builder.MinCoords = MinCoords;
			builder.MaxCoords = MaxCoords;
			builder.NumElementsTotal = NumElements;

			double E1 = E0 / stiffnessRatio;
			builder.GetMaterialPerElementIndex = elementIdx =>
			{
				if (elementIdx[0] < 3)
				{
					return new ElasticMaterial2D(E1, v, StressState2D.PlaneStress);
				}
				else
				{
					return new ElasticMaterial2D(E0, v, StressState2D.PlaneStress);
				}
			};
			builder.Thickness = thickness;

			builder.PrescribeDisplacement(UniformDdmModelBuilder2D.BoundaryRegion.LeftSide, StructuralDof.TranslationX, 0.0);
			builder.PrescribeDisplacement(UniformDdmModelBuilder2D.BoundaryRegion.LeftSide, StructuralDof.TranslationY, 0.0);
			builder.DistributeLoadAtNodes(UniformDdmModelBuilder2D.BoundaryRegion.RightSide, StructuralDof.TranslationY, load);

			return builder;
		}
	}
}
