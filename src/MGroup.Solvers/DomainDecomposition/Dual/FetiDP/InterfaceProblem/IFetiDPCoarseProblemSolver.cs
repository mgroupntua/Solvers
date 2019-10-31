using System.Collections.Generic;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DomainDecomposition.Dual.FetiDP.Matrices;

namespace MGroup.Solvers.DomainDecomposition.Dual.FetiDP.InterfaceProblem
{
    public interface IFetiDPCoarseProblemSolver
    {
        void ClearCoarseProblemMatrix();

        Vector CreateCoarseProblemRhs(FetiDPDofSeparator dofSeparator,
            Dictionary<int, IFetiDPSubdomainMatrixManager> matrixManagers,
            Dictionary<int, Vector> fr, Dictionary<int, Vector> fbc);

        //TODO: Perhaps corner nodes of each subdomain should be stored in FetiDPDofSeparator.
        void CreateAndInvertCoarseProblemMatrix(Dictionary<int, HashSet<INode>> cornerNodesOfSubdomains, 
            FetiDPDofSeparator dofSeparator, Dictionary<int, IFetiDPSubdomainMatrixManager> matrixManagers);

        Vector MultiplyInverseCoarseProblemMatrixTimes(Vector vector);

        void ReorderCornerDofs(FetiDPDofSeparator dofSeparator);
    }
}
