using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;

namespace MGroup.Solvers.Tests
{
	public class SolverBenchmarks
    {
		public static void SuiteSparseMemoryConsumptionDebugging()
		{
			for (int rep = 0; rep < 10; ++rep)
			{
				var benchmarkBuilder = new CantileverBeam.Builder();
				//benchmarkBuilder.Length = 5.0;
				CantileverBeam benchmark = benchmarkBuilder.BuildWithQuad4Elements(2000, 100);

				// Solver
				var solverBuilder = new SuiteSparseSolver.Builder();
				using (SuiteSparseSolver solver = solverBuilder.BuildSolver(benchmark.Model))
				{
					// Structural problem provider
					var provider = new ProblemStructural(benchmark.Model, solver);

					// Linear static analysis
					var childAnalyzer = new LinearAnalyzer(benchmark.Model, solver, provider);
					var parentAnalyzer = new StaticAnalyzer(benchmark.Model, solver, provider, childAnalyzer);

					// Run the analysis
					parentAnalyzer.Initialize();
					parentAnalyzer.Solve();
				}
			}
		}
	}
}
