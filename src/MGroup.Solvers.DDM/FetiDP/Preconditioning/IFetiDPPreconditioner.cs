namespace MGroup.Solvers.DDM.FetiDP.Preconditioning
{
	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.Scaling;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

	public interface IFetiDPPreconditioner : IPreconditioner
	{
		void CalcSubdomainMatrices(int subdomainID);

		void Initialize(IComputeEnvironment environment, DistributedOverlappingIndexer lagrangeVectorIndexer,
			Func<int, SubdomainLagranges> getSubdomainLagranges, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices,
			IFetiDPScaling scaling);
	}
}
