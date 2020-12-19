using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.ConjugateGradient;
using MGroup.LinearAlgebra.Iterative.GeneralizedMinimalResidual;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.Commons;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;

namespace MGroup.Solvers.Iterative
{
    public class GmresSolver : SingleSubdomainSolverBase<CsrMatrix>
    {
        private readonly GmresAlgorithm gmresAlgorithm;
        private readonly IPreconditionerFactory preconditionerFactory;

        private bool mustUpdatePreconditioner = true;
        private IPreconditioner preconditioner;

        private GmresSolver(IModel model, GmresAlgorithm gmresAlgorithm, IPreconditionerFactory preconditionerFactory, 
            IDofOrderer dofOrderer):
            base(model, dofOrderer, new CsrAssembler(true), "GmresSolver")
        {
            this.gmresAlgorithm = gmresAlgorithm;
            this.preconditionerFactory = preconditionerFactory;
        }

        public override void HandleMatrixWillBeSet()
        {
            mustUpdatePreconditioner = true;
            preconditioner = null;
        }

        public override void Initialize() { }

        public override void PreventFromOverwrittingSystemMatrices()
        {
            // No factorization is done.
        }

        /// <summary>
        /// Solves the linear system with PCG method. If the matrix has been modified, a new preconditioner will be computed.
        /// </summary>
        public override void Solve()
        {
            var watch = new Stopwatch();
            if (linearSystem.SolutionConcrete == null) linearSystem.SolutionConcrete = linearSystem.CreateZeroVectorConcrete();
            else linearSystem.SolutionConcrete.Clear();

            // Preconditioning
            if (mustUpdatePreconditioner)
            {
                watch.Start();
                preconditioner = preconditionerFactory.CreatePreconditionerFor(linearSystem.Matrix);
                watch.Stop();
                Logger.LogTaskDuration("Calculating preconditioner", watch.ElapsedMilliseconds);
                watch.Reset();
                mustUpdatePreconditioner = false;
            }

            // Iterative algorithm
            watch.Start();
            IterativeStatistics stats = gmresAlgorithm.Solve(linearSystem.Matrix, preconditioner, linearSystem.RhsConcrete,
                linearSystem.SolutionConcrete, true, () => linearSystem.CreateZeroVector()); //TODO: This way, we don't know that x0=0, which will result in an extra b-A*0
            if (!stats.HasConverged)
            {
                throw new IterativeSolverNotConvergedException(Name + " did not converge to a solution. PCG algorithm run for"
                    + $" {stats.NumIterationsRequired} iterations and the residual norm ratio was"
                    + $" {stats.ResidualNormRatioEstimation}");
            }
            watch.Stop();
            Logger.LogTaskDuration("Iterative algorithm", watch.ElapsedMilliseconds);
            Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
            Logger.IncrementAnalysisStep();
        }

        protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
        {
            //TODO: Use a reorthogonalizetion approach when solving multiple rhs vectors. It would be even better if the CG
            //      algorithm exposed a method for solving for multiple rhs vectors.
            var watch = new Stopwatch();

            // Preconditioning
            if (mustUpdatePreconditioner)
            {
                watch.Start();
                preconditioner = preconditionerFactory.CreatePreconditionerFor(linearSystem.Matrix);
                watch.Stop();
                Logger.LogTaskDuration("Calculating preconditioner", watch.ElapsedMilliseconds);
                watch.Reset();
                mustUpdatePreconditioner = false;
            }

            // Iterative algorithm
            watch.Start();
            int systemOrder = linearSystem.Matrix.NumColumns;
            int numRhs = otherMatrix.NumColumns;
            var solutionVectors = Matrix.CreateZero(systemOrder, numRhs);
            Vector solutionVector = linearSystem.CreateZeroVectorConcrete();

            // Solve each linear system
            for (int j = 0; j < numRhs; ++j)
            {
                if (j != 0) solutionVector.Clear();

                //TODO: we should make sure this is the same type as the vectors used by this solver, otherwise vector operations
                //      in CG will be slow.
                Vector rhsVector = otherMatrix.GetColumn(j);

                IterativeStatistics stats = gmresAlgorithm.Solve(linearSystem.Matrix, preconditioner, rhsVector,
                    solutionVector, true, () => linearSystem.CreateZeroVector());

                solutionVectors.SetSubcolumn(j, solutionVector);
            }

            watch.Stop();
            Logger.LogTaskDuration("Iterative algorithm", watch.ElapsedMilliseconds);
            Logger.IncrementAnalysisStep();
            return solutionVectors;
        }

        public class Builder
        {
            public IDofOrderer DofOrderer { get; set; }
                = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

			public GmresAlgorithm GmresAlgorithm { get; set; } = new GmresAlgorithm.Builder().Build();

            public IPreconditionerFactory PreconditionerFactory { get; set; } = new IdentityPreconditioner.Factory();

            public GmresSolver BuildSolver(IModel model)
                => new GmresSolver(model, GmresAlgorithm, PreconditionerFactory, DofOrderer);
        }
    }
}
