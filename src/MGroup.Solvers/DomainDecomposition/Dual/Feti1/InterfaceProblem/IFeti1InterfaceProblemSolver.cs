using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Logging.DomainDecomposition;
using MGroup.Solvers.DomainDecomposition.Dual.Feti1.Projection;
using MGroup.Solvers.DomainDecomposition.Dual.Pcg;

namespace MGroup.Solvers.DomainDecomposition.Dual.Feti1.InterfaceProblem
{
    public interface IFeti1InterfaceProblemSolver
    {
        Vector CalcLagrangeMultipliers(Feti1FlexibilityMatrix flexibility, IFetiPreconditioner preconditioner,
            Feti1Projection projection, Vector disconnectedDisplacements, Vector rigidBodyModesWork, double globalForcesNorm,
            ISolverLogger logger);
    }
}
