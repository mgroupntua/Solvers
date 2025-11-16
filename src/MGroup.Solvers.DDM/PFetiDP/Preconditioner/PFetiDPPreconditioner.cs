namespace MGroup.Solvers.DDM.PFetiDP.Preconditioner
{
	using System.Collections.Concurrent;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.MSolve.Solution.LinearSystem;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.Mappings;
	using MGroup.Solvers.DDM.PFetiDP.Dofs;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;
	using MGroup.Solvers.DDM.PSM.Preconditioning;
	using MGroup.Solvers.DDM.PSM.Scaling;

	public class PFetiDPPreconditioner : IPsmPreconditioner, IPreconditioner
	{
		private readonly IComputeEnvironment environment;
		private readonly IFetiDPCoarseProblem coarseProblem;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getFetiDPSubdomainMatrices;
		private readonly Func<int, PFetiDPSubdomainDofs> getPFetiDPSubdomainDofs;
		private readonly Func<DistributedOverlappingIndexer> getIndexer;
		private readonly IBoundaryDofScaling scaling;

		public PFetiDPPreconditioner(IComputeEnvironment environment, Func<DistributedOverlappingIndexer> getBoundaryDofIndexer,
			IBoundaryDofScaling scaling, Func<int, IFetiDPSubdomainMatrixManager> getFetiDPSubdomainMatrices,
			IFetiDPCoarseProblem coarseProblem, Func<int, PFetiDPSubdomainDofs> getPFetiDPSubdomainDofs)
		{
			this.environment = environment;
			this.getIndexer = getBoundaryDofIndexer;
			this.scaling = scaling;
			this.getFetiDPSubdomainMatrices = getFetiDPSubdomainMatrices;
			this.coarseProblem = coarseProblem;
			this.getPFetiDPSubdomainDofs = getPFetiDPSubdomainDofs;
		}

		public IPreconditioner Preconditioner => this;

		public void SolveLinearSystem(IReadOnlyVector input, IVector output)
		{
			DistributedOverlappingIndexer indexer = getIndexer();
			DistributedOverlappingVector ybe = indexer.CastCompatibleVector(input);
			DistributedOverlappingVector xbe = indexer.CastCompatibleVector(output);
			xbe.Clear();

			// Operations before coarse problem solution
			var v3e = new ConcurrentDictionary<int, Vector>(); // will be reused after coarse problem solution
			var yce = new ConcurrentDictionary<int, Vector>();
			environment.DoPerNode(subdomainID =>
			{
				IFetiDPSubdomainMatrixManager fetiDPMatrices = getFetiDPSubdomainMatrices(subdomainID);
				PFetiDPSubdomainDofs pfetiDPDofs = getPFetiDPSubdomainDofs(subdomainID);
				DiagonalMatrix Wb = scaling.SubdomainMatricesWb[subdomainID];
				IMappingMatrix Ncb = pfetiDPDofs.MatrixNcb;
				IMappingMatrix Nrb = pfetiDPDofs.MatrixNrb;

				Vector yb = ybe.LocalVectors[subdomainID];
				Vector v1 = Wb.Multiply(yb);
				Vector v2 = Ncb.Multiply(v1, false);
				Vector v3 = fetiDPMatrices.MultiplyInverseKrrTimes(Nrb.Multiply(v1, false));
				Vector v4 = fetiDPMatrices.MultiplyKcrTimes(v3);
				Vector yc = v2 - v4;

				v3e[subdomainID] = v3;
				yce[subdomainID] = yc;
			});

			// Coarse problem solution
			var xce = environment.CalcNodeData(s => Vector.CreateZero(yce[s].Length));
			coarseProblem.SolveCoarseProblem(yce, xce);

			// Operations after coarse problem solution
			environment.DoPerNode(subdomainID =>
			{
				IFetiDPSubdomainMatrixManager fetiDPMatrices = getFetiDPSubdomainMatrices(subdomainID);
				DiagonalMatrix Wb = scaling.SubdomainMatricesWb[subdomainID];
				IMappingMatrix Nrb = getPFetiDPSubdomainDofs(subdomainID).MatrixNrb;

				Vector xc = xce[subdomainID];
				Vector v3 = v3e[subdomainID];
				Vector v6 = fetiDPMatrices.MultiplyInverseKrrTimes(fetiDPMatrices.MultiplyKrcTimes(xc));
				Vector v7 = v3 - v6;
				Vector v8 = Wb.Multiply(Nrb.Multiply(v7, true));

				xbe.LocalVectors[subdomainID] = v8;
			});
			xbe.SumOverlappingEntries();

			environment.DoPerNode(subdomainID =>
			{
				IMappingMatrix Ncb = getPFetiDPSubdomainDofs(subdomainID).MatrixNcb;

				Vector xc = xce[subdomainID];
				Vector v5 = Ncb.Multiply(xc, true);

				xbe.LocalVectors[subdomainID].AddIntoThis(v5);
			});
		}

		public void Calculate(IComputeEnvironment environment, DistributedOverlappingIndexer boundaryDofIndexer, 
			IPsmInterfaceProblemMatrix interfaceProblemMatrix)
		{
			// Do nothing
		}

		public IPreconditioner CopyWithInitialSettings() => throw new NotImplementedException();

		public void UpdateMatrix(IReadOnlyMatrix matrix, bool isPatternModified) => throw new NotImplementedException();
	}
}
