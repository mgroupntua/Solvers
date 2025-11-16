namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.SchurComplements;
	using MGroup.LinearAlgebra.SchurComplements.SubmatrixExtractors;
	using MGroup.LinearAlgebra.Triangulation;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DDM.Commons;
	using MGroup.Solvers.DDM.LinearSystem;
	using MGroup.Solvers.DDM.PSM.Dofs;

	public class PsmSubdomainMatrixManagerCsc : IPsmSubdomainMatrixManager
	{
		private readonly SubdomainLinearSystem<CsrMatrix> linearSystem;
		private readonly IImplementationProvider provider;
		private readonly PsmSubdomainDofs subdomainDofs;
		private readonly SubmatrixExtractorCsrCsc submatrixExtractor = new SubmatrixExtractorCsrCsc();

		private CsrMatrix Kbb;
		private CsrMatrix Kbi;
		private CsrMatrix Kib;
		private CscMatrix Kii;
		private ILUCscFactorization inverseKii;

		public PsmSubdomainMatrixManagerCsc(
			IImplementationProvider provider, SubdomainLinearSystem<CsrMatrix> linearSystem, PsmSubdomainDofs subdomainDofs)
		{
			this.provider = provider;
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
		}

		public bool IsEmpty => inverseKii == null;

		public IReadOnlyMatrix CalcSchurComplement()
		{
			//TODO: Implement a ScurComplement class where A11 is in CSR format
			Matrix kbb = Kbb.CopyToFullMatrix();
			return SchurComplementFullCsrCsrCsc.CalcSchurComplement(kbb, Kbi, Kib, inverseKii);
		}

		public void ClearSubMatrices()
		{
			if (inverseKii != null)
			{
				inverseKii.Dispose();
			}

			inverseKii = null;
			Kii = null;
			Kbb = null;
			Kbi = null;
			Kib = null;
		}

		//TODO: Optimize this method. It is too slow.
		public void ExtractKiiKbbKib()
		{
			int[] boundaryDofs = subdomainDofs.DofsBoundaryToFree;
			int[] internalDofs = subdomainDofs.DofsInternalToFree;

			CsrMatrix Kff = linearSystem.Matrix;
			submatrixExtractor.ExtractSubmatrices(Kff, boundaryDofs, internalDofs);
			Kbb = submatrixExtractor.Submatrix00;
			Kbi = submatrixExtractor.Submatrix01;
			Kib = submatrixExtractor.Submatrix10;
			Kii = submatrixExtractor.Submatrix11;
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractor.Clear();
		}

		public void InvertKii()
		{
			if (inverseKii != null)
			{
				inverseKii.Dispose();
			}

			inverseKii = provider.CreateLUTriangulation();
			inverseKii.Factorize(Kii);
			Kii = null; // This memory is not overwritten, but it is not needed anymore either.
		}

		public Vector MultiplyInverseKii(Vector vector) => inverseKii.SolveLinearSystem(vector);

		public Vector MultiplyKbb(Vector vector) => Kbb * vector;

		public Vector MultiplyKbi(Vector vector) => Kbi.Multiply(vector, false);

		public Vector MultiplyKib(Vector vector) => Kib.Multiply(vector, false);

		public void ReorderInternalDofs() => subdomainDofs.ReorderInternalDofs(DofPermutation.CreateNoPermutation());

		public class Factory : IPsmSubdomainMatrixManagerFactory<CsrMatrix>
		{
			public Factory(double luPivotTolerance)
			{
			}

			public ISubdomainMatrixAssembler<CsrMatrix> CreateAssembler() => new CsrMatrixAssembler(false);

			public IPsmSubdomainMatrixManager CreateMatrixManager(IImplementationProvider provider,
				SubdomainLinearSystem<CsrMatrix> linearSystem, PsmSubdomainDofs subdomainDofs)
				=> new PsmSubdomainMatrixManagerCsc(provider, linearSystem, subdomainDofs);
		}
	}
}
