namespace MGroup.Solvers.DDM.FetiDP.InterfaceProblem
{
	using MGroup.LinearAlgebra.Iterative;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;

	public interface IFetiDPInterfaceProblemSolverFactory
	{
		int MaxIterations { get; set; }

		double ResidualTolerance { get; set; }

		bool ThrowExceptionIfNotConvergence { get; set; }

		bool UseObjectiveConvergenceCriterion { get; set; }

		ISystemSolutionIterativeMethod BuildIterativeMethod(IPcgResidualConvergence convergenceCriterion);
	}
}
