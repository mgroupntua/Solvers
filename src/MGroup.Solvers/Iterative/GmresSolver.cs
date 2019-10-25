using System;
using System.Diagnostics;
using MGroup.LinearAlgebra.Iterative;
using MGroup.LinearAlgebra.Iterative.GeneralizedMinimalResidual;
using MGroup.LinearAlgebra.Iterative.Preconditioning;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.Interfaces;
using MGroup.MSolve.Solvers;
using MGroup.MSolve.Solvers.Commons;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.Commons;
using MGroup.Solvers.Ordering;
using MGroup.Solvers.Ordering.Reordering;

namespace MGroup.Solvers.Iterative
{
    public class GmresSolver : SingleSubdomainNonSymmetricSolverBase<CsrMatrix>
    {
        private readonly GmresAlgorithm gmresAlgorithm;
        private readonly IPreconditionerFactory preconditionerFactory;

        private bool mustUpdatePreconditioner = true;
        private IPreconditioner preconditioner;

        public GmresSolver(IAsymmetricModel model, GmresAlgorithm gmresAlgorithm,
            IPreconditionerFactory preconditionerFactory, AsymmetricDofOrderer dofRowOrderer, IDofOrderer dofColOrderer)
            : base(model, dofRowOrderer, dofColOrderer, new CsrNonSymmetricAssembler(true), "GmresSolver")
        {
            this.gmresAlgorithm = new GmresAlgorithm.Builder().Build();
            this.preconditionerFactory = preconditionerFactory;
        }

        public override void Initialize()
        {
        }

        public override void HandleMatrixWillBeSet()
        {
            mustUpdatePreconditioner = true;
            preconditioner = null;
        }

        public override void PreventFromOverwrittingSystemMatrices()
        {
        }

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

            watch.Start();
            IterativeStatistics stats = gmresAlgorithm.Solve(linearSystem.Matrix, preconditioner, linearSystem.RhsConcrete,
                linearSystem.SolutionConcrete, true, () => linearSystem.CreateZeroVector());
            if (!stats.HasConverged)
                throw new IterativeSolverNotConvergedException("Gmres did not converge");
            watch.Stop();
            Logger.LogTaskDuration("Iterative algorithm", watch.ElapsedMilliseconds);
            Logger.LogIterativeAlgorithm(stats.NumIterationsRequired, stats.ResidualNormRatioEstimation);
            Logger.IncrementAnalysisStep();
        }

        protected override Matrix InverseSystemMatrixTimesOtherMatrix(IMatrixView otherMatrix)
        {
            throw new NotImplementedException();
        }

        public class Builder : ISolverBuilder
        {
            public AsymmetricDofOrderer RowDofOrderer { get; set; } =
                new AsymmetricDofOrderer(new RowDofOrderingStrategy());

            public IDofOrderer ColumnDofOrderer { get; set; } =
                new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());

            public GmresAlgorithm GmresAlgorithm { get; set; } = (new GmresAlgorithm.Builder()).Build();

            public IPreconditionerFactory PreconditionerFactory { get; set; } = new IdentityPreconditioner.Factory();

            public ISolver BuildSolver(IModel model)
            {
                if (!(model is IAsymmetricModel asymmetricModel))
                    throw new ArgumentException("Gmres solver builder can be used only with asymmetric models.");

                return new GmresSolver(asymmetricModel, GmresAlgorithm,PreconditionerFactory, RowDofOrderer, ColumnDofOrderer);
            }
        }
    }
}
