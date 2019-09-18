using System;
using System.Collections.Generic;
using System.Diagnostics;
using MGroup.LinearAlgebra.Iterative.ConjugateGradient;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Vectors;

//TODO: According to Fragakis PhD this is valid only for Lumped preconditioner. For other preconditioners we need to isolate
//      sum(Bpb * Kbb * Bpb^T) * residual
namespace MGroup.Solvers.DomainDecomposition.Dual.Pcg
{
    public class ApproximateResidualConvergence : IFetiPcgConvergence
    {
        private readonly double globalForcesNorm;

        private ApproximateResidualConvergence(double globalForcesNorm)
        {
            this.globalForcesNorm = globalForcesNorm;
        }

        public double EstimateResidualNormRatio(PcgAlgorithmBase pcg)
        {
            Debug.Assert(globalForcesNorm != 0.0, "norm2(globalForces) must be set first");
            return pcg.PrecondResidual.Norm2() / globalForcesNorm;
        }

        public double EstimateResidualNormRatio(IVectorView lagrangeMultipliers, IVectorView projectedPrecondResidual)
        {
            Debug.Assert(globalForcesNorm != 0.0, "norm2(globalForces) must be set first");
            return projectedPrecondResidual.Norm2() / globalForcesNorm;
        }

        public void Initialize(PcgAlgorithmBase pcg) { } // Do nothing

        public class Factory : IFetiPcgConvergenceFactory
        {
            public IFetiPcgConvergence CreateConvergenceStrategy(double globalForcesNorm)
                => new ApproximateResidualConvergence(globalForcesNorm);
        }
    }
}
