namespace MGroup.Solvers.DDM.PFetiDP.Preconditioner
{
	using System.Collections.Concurrent;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Iterative.Preconditioning;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.DDM.FetiDP.CoarseProblem;
	using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;
	using MGroup.Solvers.DDM.Mappings;
	using MGroup.Solvers.DDM.PFetiDP.Dofs;
	using MGroup.Solvers.DDM.PSM.InterfaceProblem;
	using MGroup.Solvers.DDM.PSM.Preconditioning;
	using MGroup.Solvers.DDM.PSM.Scaling;

	public class PFetiDPPreconditionerExplicit : IPsmPreconditioner, IPreconditioner
	{
		private readonly IComputeEnvironment environment;
		private readonly IFetiDPCoarseProblem coarseProblem;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getFetiDPSubdomainMatrices;
		private readonly Func<int, PFetiDPSubdomainDofs> getPFetiDPSubdomainDofs;
		private readonly Func<DistributedOverlappingIndexer> getIndexer;
		private readonly IBoundaryDofScaling scaling;

		//TODO: some rows will definitely be 0 and some entries might be 0, depending on the case. I should use a block row major format or CSR.
		private readonly ConcurrentDictionary<int, FullMatrixRowMajor> matricesWb_Nbr_invKrr_Krc 
			= new ConcurrentDictionary<int, FullMatrixRowMajor>();

		public PFetiDPPreconditionerExplicit(IComputeEnvironment environment, Func<DistributedOverlappingIndexer> getBoundaryDofIndexer,
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

		public void SolveLinearSystem(IVectorView input, IVector output)
		{
			throw new NotImplementedException();
			//DistributedOverlappingVector ybe = getIndexer().CheckCompatibleVector(input);
			//DistributedOverlappingVector xbe = getIndexer().CheckCompatibleVector(output);
			//xbe.Clear();

			//// Operations before coarse problem solution
			//var v3e = new ConcurrentDictionary<int, Vector>(); // will be reused after coarse problem solution
			//var yce = new ConcurrentDictionary<int, Vector>();
			//environment.DoPerNode(subdomainID =>
			//{
			//	IFetiDPSubdomainMatrixManager fetiDPMatrices = getFetiDPSubdomainMatrices(subdomainID);
			//	PFetiDPSubdomainDofs pfetiDPDofs = getPFetiDPSubdomainDofs(subdomainID);
			//	DiagonalMatrix Wb = scaling.SubdomainMatricesWb[subdomainID];
			//	IMappingMatrix Ncb = pfetiDPDofs.MatrixNcb;
			//	IMappingMatrix Nrb = pfetiDPDofs.MatrixNrb;

			//	Vector yb = ybe.LocalVectors[subdomainID];
			//	Vector v1 = Wb.Multiply(yb);
			//	Vector v2 = Ncb.Multiply(v1, false);
			//	Vector v3 = fetiDPMatrices.MultiplyInverseKrrTimes(Nrb.Multiply(v1, false));
			//	Vector v4 = fetiDPMatrices.MultiplyKcrTimes(v3);
			//	Vector yc = v2 - v4;

			//	v3e[subdomainID] = v3;
			//	yce[subdomainID] = yc;
			//});

			//// Coarse problem solution
			//var xce = environment.CalcNodeData(s => Vector.CreateZero(yce[s].Length));
			//coarseProblem.SolveCoarseProblem(yce, xce);

			//// Operations after coarse problem solution
			//environment.DoPerNode(subdomainID =>
			//{
			//	IFetiDPSubdomainMatrixManager fetiDPMatrices = getFetiDPSubdomainMatrices(subdomainID);
			//	DiagonalMatrix Wb = scaling.SubdomainMatricesWb[subdomainID];
			//	IMappingMatrix Nrb = getPFetiDPSubdomainDofs(subdomainID).MatrixNrb;

			//	Vector xc = xce[subdomainID];
			//	Vector v3 = v3e[subdomainID];
			//	Vector v6 = fetiDPMatrices.MultiplyInverseKrrTimes(fetiDPMatrices.MultiplyKrcTimes(xc));
			//	Vector v7 = v3 - v6;
			//	Vector v8 = Wb.Multiply(Nrb.Multiply(v7, true));

			//	xbe.LocalVectors[subdomainID] = v8;
			//});
			//xbe.SumOverlappingEntries();

			//environment.DoPerNode(subdomainID =>
			//{
			//	IMappingMatrix Ncb = getPFetiDPSubdomainDofs(subdomainID).MatrixNcb;

			//	Vector xc = xce[subdomainID];
			//	Vector v5 = Ncb.Multiply(xc, true);

			//	xbe.LocalVectors[subdomainID].AddIntoThis(v5);
			//});
		}

		public void Calculate(IComputeEnvironment environment, DistributedOverlappingIndexer boundaryDofIndexer, 
			IPsmInterfaceProblemMatrix interfaceProblemMatrix)
		{
			// Do nothing
			environment.DoPerNode(subdomainID =>
			{
				IFetiDPSubdomainMatrixManager fetiDPMatrices = getFetiDPSubdomainMatrices(subdomainID);
				PFetiDPSubdomainDofs pfetiDPDofs = getPFetiDPSubdomainDofs(subdomainID);
				DiagonalMatrix Wb = scaling.SubdomainMatricesWb[subdomainID];
				var Nrb = (MappingMatrixN)(pfetiDPDofs.MatrixNrb);

				Matrix invKrr_Krc = fetiDPMatrices.CalcInvKrrTimesKrc();
				FullMatrixRowMajor result = SelectAndScaleRows(Wb, Nrb, invKrr_Krc);

				matricesWb_Nbr_invKrr_Krc[subdomainID] = result;
			});
		}

		public IPreconditioner CopyWithInitialSettings() => throw new NotImplementedException();

		public void UpdateMatrix(IMatrixView matrix, bool isPatternModified) { }

		private static FullMatrixRowMajor SelectAndScaleRows(DiagonalMatrix Wb, MappingMatrixN Nrb, Matrix invKrr_Krc)
		{
			var result = FullMatrixRowMajor.CreateZero(Nrb.NumColumns, invKrr_Krc.NumColumns);
			foreach (var nonZeroEntry in Nrb.RowsToColumns)
			{
				int remainderDof = nonZeroEntry.Key;
				int boundaryDof = nonZeroEntry.Value;

				Vector row = invKrr_Krc.GetRow(remainderDof); //TODO: directly copy to the final array using an offset
				row.ScaleIntoThis(Wb.RawDiagonal[boundaryDof]);
				result.SetRow(boundaryDof, row);
			}
			return result;
		}
	}
}
