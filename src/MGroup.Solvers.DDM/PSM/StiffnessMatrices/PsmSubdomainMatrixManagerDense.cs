namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DDM.Commons;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.PSM.Dofs;

	public class PsmSubdomainMatrixManagerDense : IPsmSubdomainMatrixManager
	{
		private readonly SubdomainLinearSystem<Matrix> linearSystem;
		private readonly PsmSubdomainDofs subdomainDofs;

		private Matrix Kbb;
		private Matrix Kbi;
		private Matrix Kib;
		private Matrix Kii;
		private Matrix inverseKii;

		public PsmSubdomainMatrixManagerDense(SubdomainLinearSystem<Matrix> linearSystem, PsmSubdomainDofs subdomainDofs)
		{
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

		public bool IsEmpty => inverseKii == null;

		public IReadOnlyMatrix CalcSchurComplement() => Kbb - Kbi * (inverseKii * Kib);

		public void ClearSubMatrices()
		{
			Kbb = null;
			Kbi = null;
			Kib = null;
			Kii = null;
			inverseKii = null;
		}

		public void ExtractKiiKbbKib()
		{
			int[] boundaryDofs = subdomainDofs.DofsBoundaryToFree;
			int[] internalDofs = subdomainDofs.DofsInternalToFree;
			Matrix Kff = linearSystem.Matrix;
			Kbb = Kff.GetSubmatrix(boundaryDofs, boundaryDofs);
			Kbi = Kff.GetSubmatrix(boundaryDofs, internalDofs);
			Kib = Kff.GetSubmatrix(internalDofs, boundaryDofs);
			Kii = Kff.GetSubmatrix(internalDofs, internalDofs);
		}

		public void HandleDofsWereModified() => ClearSubMatrices();

		public void InvertKii()
		{
			inverseKii = Kii.Invert();
			Kii = null; // Kii has been overwritten
		}

		public Vector MultiplyInverseKii(Vector vector) => inverseKii * vector;

		public Vector MultiplyKbb(Vector vector) => Kbb * vector;

		public Vector MultiplyKbi(Vector vector) => Kbi * vector;

		public Vector MultiplyKib(Vector vector) => Kib * vector;

		public void ReorderInternalDofs() => subdomainDofs.ReorderInternalDofs(DofPermutation.CreateNoPermutation());

		public class Factory : IPsmSubdomainMatrixManagerFactory<Matrix>
		{
			public ISubdomainMatrixAssembler<Matrix> CreateAssembler() => new DenseMatrixAssembler();

			public IPsmSubdomainMatrixManager CreateMatrixManager(IImplementationProvider provider,
				SubdomainLinearSystem<Matrix> linearSystem, PsmSubdomainDofs subdomainDofs)
				=> new PsmSubdomainMatrixManagerDense(linearSystem, subdomainDofs);
		}
	}
}
