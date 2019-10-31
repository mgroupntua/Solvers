using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Logging.DomainDecomposition;
using MGroup.Solvers.DomainDecomposition.Dual.Pcg;

//TODO: This could be split into an interface with the same name and an IFtiDPCoarseProblemSolver.
namespace MGroup.Solvers.DomainDecomposition.Dual.FetiDP.InterfaceProblem
{
    public interface IFetiDPInterfaceProblemSolver
    {
        (Vector lagrangeMultipliers, Vector cornerDisplacements) SolveInterfaceProblem(FetiDPFlexibilityMatrix flexibility, 
            IFetiPreconditioner preconditioner, IFetiDPCoarseProblemSolver coarseProblemSolver, Vector globalFcStar, Vector dr,
            double globalForcesNorm, ISolverLogger logger);
    }
}
