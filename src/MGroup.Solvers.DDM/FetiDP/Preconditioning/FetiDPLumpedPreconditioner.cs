namespace MGroup.Solvers.DDM.FetiDP.Preconditioning
{
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Matrices.Operators;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.Scaling;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

	public class FetiDPLumpedPreconditioner : IFetiDPPreconditioner
	{
		private IComputeEnvironment environment;
		private DistributedOverlappingIndexer lagrangeVectorIndexer;
		private Func<int, SubdomainLagranges> getSubdomainLagranges;
		private Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;
		private IFetiDPScaling scaling;

		public void SolveLinearSystem(IVectorView input, IVector output)
		{
			DistributedOverlappingVector ye = lagrangeVectorIndexer.CastCompatibleVector(input);
			DistributedOverlappingVector xe = lagrangeVectorIndexer.CastCompatibleVector(output);
			//xe.Clear(); //TODO: Clear the existing local vectors instead of reallocating them

			environment.DoPerNode(s =>
			{
				IFetiDPSubdomainMatrixManager fetiDPMatrices = getSubdomainMatrices(s);
				SignedBooleanMatrixRowMajor Dbr = getSubdomainLagranges(s).MatrixDbr;
				DiagonalMatrix Wbr = scaling.SubdomainMatricesWbr[s];

				// Dbr * Wbr * Kbb * Wbr * Dbr^T * y
				Vector ys = ye.LocalVectors[s];
				Vector v1 = Wbr.Multiply(Dbr.Multiply(ys, true));
				Vector v2 = fetiDPMatrices.MultiplyKbbTimes(v1);
				xe.LocalVectors[s] = Dbr.Multiply(Wbr.Multiply(v2));
			});
			xe.SumOverlappingEntries();
		}

		public void CalcSubdomainMatrices(int subdomainID)
		{
			IFetiDPSubdomainMatrixManager subdomainMatrices = getSubdomainMatrices(subdomainID);
			subdomainMatrices.ExtractKiiKbbKib(); //TODO: Only extract Kbb
		}

		public IPreconditioner CopyWithInitialSettings() => new FetiDPLumpedPreconditioner();

		public void Initialize(IComputeEnvironment environment, DistributedOverlappingIndexer lagrangeVectorIndexer,
			Func<int, SubdomainLagranges> getSubdomainLagranges, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices,
			IFetiDPScaling scaling)
		{
			this.environment = environment;
			this.lagrangeVectorIndexer = lagrangeVectorIndexer;
			this.getSubdomainLagranges = getSubdomainLagranges;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.scaling = scaling;
		}

		public void UpdateMatrix(IMatrixView matrix, bool isPatternModified) { }
	}
}
