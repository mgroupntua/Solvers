using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Matrices.Operators;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DomainDecomposition.Dual.LagrangeMultipliers;
using MGroup.Solvers.DomainDecomposition.Dual.Pcg;
using MGroup.Solvers.DomainDecomposition.Dual.StiffnessDistribution;

namespace MGroup.Solvers.DomainDecomposition.Dual.Preconditioning
{
    public class DirichletPreconditioner : IFetiPreconditioner
    {
        private readonly Dictionary<int, IFetiSubdomainMatrixManager> matrixManagers;
        private readonly Dictionary<int, IMappingMatrix> preconditioningBoundarySignedBooleanMatrices;
        private readonly int[] subdomainIDs;

        private DirichletPreconditioner(int[] subdomainIDs, Dictionary<int, IFetiSubdomainMatrixManager> matrixManagers, 
            Dictionary<int, IMappingMatrix> preconditioningBoundarySignedBooleanMatrices)
        {
            this.subdomainIDs = subdomainIDs;
            this.matrixManagers = matrixManagers;
            this.preconditioningBoundarySignedBooleanMatrices = preconditioningBoundarySignedBooleanMatrices;
        }

        public void SolveLinearSystem(Vector rhs, Vector lhs)
        {
            lhs.Clear(); //TODO: this should be avoided
            foreach (int s in subdomainIDs)
            {
                IFetiSubdomainMatrixManager matrixManager = matrixManagers[s];
                IMappingMatrix Bpb = preconditioningBoundarySignedBooleanMatrices[s];

                // inv(F) * y = Bpb * S * Bpb^T * y
                // S = Kbb - Kbi * inv(Kii) * Kib
                Vector By = Bpb.Multiply(rhs, true);
                Vector temp = matrixManager.MultiplyKibTimes(By);
                temp = matrixManager.MultiplyInverseKiiTimes(temp);
                temp = matrixManager.MultiplyKbiTimes(temp);
                temp = matrixManager.MultiplyKbbTimes(By) - temp;
                Vector subdomainContribution = Bpb.Multiply(temp);
                lhs.AddIntoThis(subdomainContribution);
            }
        }

        public void SolveLinearSystems(Matrix rhs, Matrix lhs)
        {
            lhs.Clear(); //TODO: this should be avoided
            foreach (int s in subdomainIDs)
            {
                IFetiSubdomainMatrixManager matrixManager = matrixManagers[s];
                IMappingMatrix Bpb = preconditioningBoundarySignedBooleanMatrices[s];

                // inv(F) * Y = Bpb * S * Bpb^T * Y
                // S = Kbb - Kbi * inv(Kii) * Kib
                Matrix BY = Bpb.MultiplyRight(rhs, true);
                Matrix temp = matrixManager.MultiplyKibTimes(BY);
                temp = matrixManager.MultiplyInverseKiiTimes(temp);
                temp = matrixManager.MultiplyKbiTimes(temp);
                temp = matrixManager.MultiplyKbbTimes(BY) - temp;
                Matrix subdomainContribution = Bpb.MultiplyRight(temp);
                lhs.AddIntoThis(subdomainContribution);
            }
        }

        public class Factory : FetiPreconditionerFactoryBase
        {
            public override bool ReorderInternalDofsForFactorization => true;

            public override IFetiPreconditioner CreatePreconditioner(IModel model,
                IStiffnessDistribution stiffnessDistribution, IDofSeparator dofSeparator,
                ILagrangeMultipliersEnumerator lagrangeEnumerator, Dictionary<int, IFetiSubdomainMatrixManager> matrixManagers)
            {
                IReadOnlyList<ISubdomain> subdomains = model.Subdomains;
                int[] subdomainIDs = dofSeparator.BoundaryDofIndices.Keys.ToArray();
                Dictionary<int, IMappingMatrix> boundaryBooleans = CalcBoundaryPreconditioningBooleanMatrices(
                    stiffnessDistribution, dofSeparator, lagrangeEnumerator);

                foreach (int s in subdomainIDs)
                {
                    if (!subdomains[s].StiffnessModified) continue;
                    Debug.WriteLine($"{typeof(DirichletPreconditioner).Name}.{this.GetType().Name}:"
                        + $" Extracting boundary/internal submatrices of subdomain {s} for preconditioning");
                    IFetiSubdomainMatrixManager matrixManager = matrixManagers[s];
                    int[] boundaryDofs = dofSeparator.BoundaryDofIndices[s];
                    int[] internalDofs = dofSeparator.InternalDofIndices[s];
                    matrixManager.ExtractKbb(boundaryDofs);
                    matrixManager.ExtractKbiKib(boundaryDofs, internalDofs);
                    matrixManager.ExtractAndInvertKii(internalDofs);
                }
                
                return new DirichletPreconditioner(subdomainIDs, matrixManagers, boundaryBooleans);
            }
        }
    }
}
