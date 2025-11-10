namespace MGroup.Solvers.DDM.FetiDP.InterfaceProblem
{
	using System.Diagnostics;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Solvers.DDM.FetiDP.Vectors;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;

	/// <summary>
	/// At each iteration, it checks if ||K * u - f|| / ||f|| &lt; tol, where all vectors and matrices correspond to the free 
	/// dofs of the whole model. Since this is very inefficient, it is recommended to use this class only for testing, 
	/// comparison or benchmarking purposes.
	/// </summary>
	public class ObjectiveConvergenceCriterion<TMatrix> : IPcgResidualConvergence
		where TMatrix: class, IMatrix
	{
		public static bool optimizationsForSymmetricCscMatrix = false;
		private readonly IComputeEnvironment environment;
		private readonly DistributedAlgebraicModel<TMatrix> algebraicModel;
		private readonly FetiDPSolutionRecovery solutionRecovery;

		private double normF0;
		private DistributedOverlappingMatrix<CsrMatrix> KffCsr;

		public ObjectiveConvergenceCriterion(IComputeEnvironment environment, DistributedAlgebraicModel<TMatrix> algebraicModel,
			FetiDPSolutionRecovery solutionRecovery)
		{
			this.environment = environment;
			this.algebraicModel = algebraicModel;
			this.solutionRecovery = solutionRecovery;
		}

		public long EllapsedMilliseconds { get; set; } = 0;

		public IPcgResidualConvergence CopyWithInitialSettings()
			=> new ObjectiveConvergenceCriterion<TMatrix>(environment, algebraicModel, solutionRecovery);

		public double EstimateResidualNormRatio(PcgAlgorithmBase pcg)
		{
			var watch = new Stopwatch();

			// Find solution at free dofs
			watch.Start();
			var lambda = (DistributedOverlappingVector)(pcg.Solution);
			var Uf = new DistributedOverlappingVector(algebraicModel.FreeDofIndexer);
			solutionRecovery.CalcPrimalSolution(lambda, Uf);

			IVector Ff = algebraicModel.LinearSystem.RhsVector;
			IVector residual = Ff.CreateZeroVectorWithSameFormat();

			if (optimizationsForSymmetricCscMatrix)
			{
				if (KffCsr == null)
				{
					KffCsr = CopyKffToCsr();
				}
				KffCsr.MultiplyIntoResult(Uf, residual);
			}
			else
			{
				IMatrix Kff = algebraicModel.LinearSystem.Matrix;
				Kff.MultiplyIntoResult(Uf, residual);
			}

			residual.LinearCombinationIntoThis(-1.0, Ff, +1.0);
			double result = residual.Norm2() / normF0;
			watch.Stop();

			this.EllapsedMilliseconds += watch.ElapsedMilliseconds;

			//Console.WriteLine($"Residual norm ratio = {result}");
			return result;
		}

		public void Initialize(PcgAlgorithmBase pcg)
		{
			normF0 = algebraicModel.LinearSystem.RhsVector.Norm2();

			if (optimizationsForSymmetricCscMatrix)
			{
				// Just reset the matrix here, so that its creation can be done in a section of code that will be timed.
				KffCsr = null;
			}
		}

		private DistributedOverlappingMatrix<CsrMatrix> CopyKffToCsr()
		{
			var indexer = algebraicModel.FreeDofIndexer;
			var KffCsr = new DistributedOverlappingMatrix<CsrMatrix>(indexer);
			environment.DoPerNode(subdomainID =>
			{
				TMatrix Kffs = algebraicModel.LinearSystem.Matrix.LocalMatrices[subdomainID];
				var KffsCasted = Kffs as SymmetricCscMatrix;
				KffCsr.LocalMatrices[subdomainID] = KffsCasted.ConvertToCsr();
			});
			return KffCsr;
		}
	}
}
