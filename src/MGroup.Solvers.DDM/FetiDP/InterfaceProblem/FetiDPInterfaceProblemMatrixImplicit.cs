namespace MGroup.Solvers.DDM.FetiDP.InterfaceProblem
{
	using System.Collections.Concurrent;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Matrices.Operators;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

	public class FetiDPInterfaceProblemMatrixImplicit : IFetiDPInterfaceProblemMatrix
	{
		private readonly IFetiDPCoarseProblem coarseProblem;
		private readonly IComputeEnvironment environment;
		private readonly IDictionary<int, SubdomainLagranges> subdomainLagranges;
		private readonly IDictionary<int, IFetiDPSubdomainMatrixManager> subdomainMatrices;

		private DistributedOverlappingIndexer lagrangeVectorIndexer;

		public FetiDPInterfaceProblemMatrixImplicit(IComputeEnvironment environment, IFetiDPCoarseProblem coarseProblem,
			IDictionary<int, SubdomainLagranges> subdomainLagranges,
			IDictionary<int, IFetiDPSubdomainMatrixManager> subdomainMatrices)
		{
			this.environment = environment;
			this.coarseProblem = coarseProblem;
			this.subdomainLagranges = subdomainLagranges;
			this.subdomainMatrices = subdomainMatrices;
		}

		public int NumColumns => lagrangeVectorIndexer.NumGlobalIndices;

		public int NumRows => lagrangeVectorIndexer.NumGlobalIndices;

		public void Calculate(DistributedOverlappingIndexer lagrangeVectorIndexer)
		{
			this.lagrangeVectorIndexer = lagrangeVectorIndexer;
		}

		public void Multiply(IReadOnlyVector input, IVector output)
		{
			DistributedOverlappingVector xe = lagrangeVectorIndexer.CastCompatibleVector(input);
			DistributedOverlappingVector ye = lagrangeVectorIndexer.CastCompatibleVector(output);
			//ye.Clear(); //TODO: Clear the existing local vectors instead of reallocating them

			// Operations before coarse problem solution
			var v1e = new ConcurrentDictionary<int, Vector>(); // will be reused after coarse problem solution
			var yce = new ConcurrentDictionary<int, Vector>();
			environment.DoPerNode(s =>
			{
				IFetiDPSubdomainMatrixManager fetiDPMatrices = subdomainMatrices[s];
				SignedBooleanMatrixRowMajor Dr = subdomainLagranges[s].MatrixDr;

				Vector xs = xe.LocalVectors[s];
				Vector v1s = fetiDPMatrices.MultiplyInverseKrrTimes(Dr.Multiply(xs, true));
				Vector ycs = fetiDPMatrices.MultiplyKcrTimes(v1s);

				v1e[s] = v1s;
				yce[s] = ycs;
			});

			// Coarse problem solution
			var xce = environment.CalcNodeData(s => Vector.CreateZero(yce[s].Length));
			coarseProblem.SolveCoarseProblem(yce, xce);

			// Operations after coarse problem solution
			environment.DoPerNode(s =>
			{
				IFetiDPSubdomainMatrixManager fetiDPMatrices = subdomainMatrices[s];
				SignedBooleanMatrixRowMajor Dr = subdomainLagranges[s].MatrixDr;

				Vector xcs = xce[s];
				Vector v2s = fetiDPMatrices.MultiplyInverseKrrTimes(fetiDPMatrices.MultiplyKrcTimes(xcs));
				v2s.AddIntoThis(v1e[s]);
				ye.LocalVectors[s] = Dr.Multiply(v2s);
			});
			ye.SumOverlappingEntries();
		}
	}
}
