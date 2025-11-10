namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;

	public interface IPsmInterfaceProblemSolverFactory
	{
		int MaxIterations { get; set; }

		double ResidualTolerance { get; set; }

		bool ThrowExceptionIfNotConvergence { get; set; }

		bool UseObjectiveConvergenceCriterion { get; set; }

		ISystemSolutionIterativeMethod BuildIterativeMethod(IPcgResidualConvergence convergenceCriterion);
	}
}
