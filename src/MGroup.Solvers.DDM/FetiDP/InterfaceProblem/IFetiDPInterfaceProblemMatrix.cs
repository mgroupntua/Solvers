namespace MGroup.Solvers.DDM.FetiDP.InterfaceProblem
{
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative;

	public interface IFetiDPInterfaceProblemMatrix : ILinearTransformation
	{
		void Calculate(DistributedOverlappingIndexer lagrangeVectorIndexer);
	}
}
