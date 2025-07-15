namespace MGroup.Solvers.DDM.FetiDP.StiffnessMatrices
{
	using MGroup.LinearAlgebra.Implementations;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Reordering;
	using MGroup.LinearAlgebra.SchurComplements;
	using MGroup.LinearAlgebra.SchurComplements.SubmatrixExtractors;
	using MGroup.LinearAlgebra.Triangulation;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Solvers.Assemblers;
	using MGroup.Solvers.DDM.Commons;
	using MGroup.Solvers.DDM.FetiDP.Dofs;
	using MGroup.Solvers.DDM.InformedPredictors;
	using MGroup.Solvers.DDM.LinearSystem;

	public class FetiDPSubdomainMatrixManagerSymmetricCscV2 : IFetiDPSubdomainMatrixManager
	{
		/// <summary>
		/// In FETI-DP Krr is also used for the preconditioner. In PFETI-DP only Krr is only used for the Schur complement of 
		/// remainder dofs.
		/// </summary>
		private readonly bool clearKrrAfterFactorization;
		private readonly SubdomainLinearSystem<SymmetricCscMatrix> linearSystem;
		private readonly IImplementationProvider provider;
		private readonly AmdSymmetricOrdering reordering;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly SubmatrixExtractorPckCsrCscSym submatrixExtractorBoundaryInternal = new SubmatrixExtractorPckCsrCscSym();
		private readonly SubmatrixExtractorPckCsrCscSym submatrixExtractorCornerRemainder = new SubmatrixExtractorPckCsrCscSym();

		private SymmetricMatrix Kbb, Kcc;
		private CsrMatrix Kbi, Kcr;
		private SymmetricCscMatrix Kii, Krr;
		private ICholeskySymmetricCsc inverseKii, inverseKrr;
		private DiagonalMatrix inverseKiiDiagonal;
		private SymmetricMatrix Scc;

		private bool UseOlds = false;
		private bool UseOldMultiplyKcrInverseKrr = false;
		private bool UseOldMultiplyInverseKrrKrc = false;

		public FetiDPSubdomainMatrixManagerSymmetricCscV2(
			IImplementationProvider provider, SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, 
			FetiDPSubdomainDofs subdomainDofs, bool clearKrrAfterFactorization)
		{
			this.provider = provider;
			this.linearSystem = linearSystem;
			this.subdomainDofs = subdomainDofs;
			this.clearKrrAfterFactorization = clearKrrAfterFactorization;
			this.reordering = new AmdSymmetricOrdering(provider);
		}

		public bool IsEmpty => inverseKrr == null;

		public IMatrix SchurComplementOfRemainderDofs => Scc;

		public IPredictionProvider SolutionPredictor { get; set; }

		public Matrix CalcInvKrrTimesKrc()
		{
			throw new NotImplementedException();
		}

		public void CalcSchurComplementOfRemainderDofs()
		{
			if (UseOlds)
			{
				CalcSchurComplementOfRemainderDofsOLD();
			}
			else
			{
				CalcSchurComplementOfRemainderDofsNEW();
			}
		}

		public void CalcSchurComplementOfRemainderDofsOLD()
		{
			Scc = SymmetricMatrix.CreateZero(Kcc.Order);
			SchurComplementPckCsrSymCsc.CalcSchurComplement(Kcc, Kcr, inverseKrr, Scc);
		}

		public void CalcSchurComplementOfRemainderDofsNEW()
		{
			int n = Kcc.Order;

			// Initialize Schur complement matrix
			Scc = SymmetricMatrix.CreateZero(n);

			// Temporary vectors
			double[] ub = new double[n];
			double[] fa = new double[inverseKrr.Order];

			// For each column of the identity
			for (int i = 0; i < n; i++)
			{
				// Set ub = e_i
				Array.Clear(ub, 0, n);
				ub[i] = 1.0;

				// Solve
				(_, Vector Fb) = CalculateResponse(ub, fa);

				// Store Fb as the i-th column (and row) of Scc
				// Because Scc is symmetric, only fill upper triangle
				for (int j = i; j < n; j++)
				{
					Scc[i, j] = Fb[j];
				}
			}
		}

		//public void CalcSchurComplementOfRemainderDofsNEW()
		//{
		//	////TODO infer only the upper diagonal part
		//	//double[] identityValuesUb = new double[Kcc.Order];
		//	//double[] zeroFa = new double[Krr.NumColumns];
		//	//for (int i = 0; i < identityValuesUb.Length; i++)
		//	//{
		//	//	identityValuesUb[i] = 1; 
		//	//	if(i>0) { identityValuesUb[i-1] = 0; }
		//	//	(Vector Ua, Vector Fb) = CalculateResponse( identityValuesUb, zeroFa);

		//	//}
		//	//Vector eye = 
		//	//Scc = SymmetricMatrix.CreateZero(Kcc.Order);
		//	//SchurComplementPckCsrSymCsc.CalcSchurComplement(Kcc, Kcr, inverseKrr, Scc);
		//}


		public void ClearSubMatrices()
		{
			if (inverseKrr != null)
			{
				inverseKrr.Dispose();
			}

			inverseKrr = null;
			Kcc = null;
			Kcr = null;
			Krr = null;
			Scc = null;

			if (inverseKii != null)
			{
				inverseKii.Dispose();
			}

			inverseKii = null;
			inverseKiiDiagonal = null;
			Kbb = null;
			Kbi = null;
			Kii = null;
		}

		public void ExtractKiiKbbKib()
		{
			int[] boundaryRemainderToRemainder = subdomainDofs.DofsBoundaryRemainderToRemainder;
			int[] internalToRemainder = subdomainDofs.DofsInternalToRemainder;

			submatrixExtractorBoundaryInternal.ExtractSubmatrices(Krr, boundaryRemainderToRemainder, internalToRemainder);
			Kbb = submatrixExtractorBoundaryInternal.Submatrix00;
			Kbi = submatrixExtractorBoundaryInternal.Submatrix01;
			Kii = submatrixExtractorBoundaryInternal.Submatrix11;
		}

		public void ExtractKrrKccKrc()
		{
			int[] cornerToFree = subdomainDofs.DofsCornerToFree;
			int[] remainderToFree = subdomainDofs.DofsRemainderToFree;

			SymmetricCscMatrix Kff = linearSystem.Matrix;
			submatrixExtractorCornerRemainder.ExtractSubmatrices(Kff, cornerToFree, remainderToFree);
			Kcc = submatrixExtractorCornerRemainder.Submatrix00;
			Kcr = submatrixExtractorCornerRemainder.Submatrix01;
			Krr = submatrixExtractorCornerRemainder.Submatrix11;

			submatrixExtractorBoundaryInternal.Clear();
		}

		public void HandleDofsWereModified()
		{
			ClearSubMatrices();
			submatrixExtractorCornerRemainder.Clear();
		}

		public void InvertKii(bool diagonalOnly)
		{
			if (diagonalOnly)
			{
				inverseKiiDiagonal = DiagonalMatrix.CreateFromArray(((IDiagonalAccessible)Kii).GetDiagonalAsArray());
				inverseKiiDiagonal.Invert();
			}
			else
			{
				if (inverseKii != null)
				{
					inverseKii.Dispose();
				}

				inverseKii = provider.CreateCholeskyTriangulation();
				inverseKii.Factorize(Kii);
			}

			Kii = null; // It has not been mutated, but it is no longer needed
		}

		public void InvertKrr()
		{
			if (inverseKrr != null)
			{
				inverseKrr.Dispose();
			}

			inverseKrr = provider.CreateCholeskyTriangulation();
			inverseKrr.Factorize(Krr);

			if (clearKrrAfterFactorization)
			{
				Krr = null; // It has not been mutated, but it is no longer needed
			}
		}

		#region new multiplications
		public Vector MultiplyKcrTimesInverseKrrTimes(Vector vector)
		{
			if (UseOldMultiplyKcrInverseKrr)
			{
				return MultiplyKcrTimesInverseKrrTimes_OLD(vector);
			}
			else
			{
				return MultiplyKcrTimesInverseKrrTimes_NEW(vector);
			}
		}

		private Vector MultiplyKcrTimesInverseKrrTimes_NEW(Vector vector)
		{
			(_, Vector Fb) = CalculateResponse(new double[Kcc.NumColumns], vector.RawData);
			return Fb;
		}

		private Vector MultiplyKcrTimesInverseKrrTimes_OLD(Vector vector)
		{
			return MultiplyKcrTimes(MultiplyInverseKrrTimes(vector));
		}

		public Vector MultiplyInverseKrrTimesKrcTimes(Vector vector)
		{
			if (UseOldMultiplyInverseKrrKrc)
			{
				return MultiplyInverseKrrTimesKrcTimes_OLD(vector);
			}
			else
			{
				return MultiplyInverseKrrTimesKrcTimes_NEW(vector);
			}
		}

		private Vector MultiplyInverseKrrTimesKrcTimes_NEW(Vector vector)
		{
			(Vector Ua, _) = CalculateResponse(vector.RawData, new double[inverseKrr.Order]);
			return Ua.Scale(-1);
		}

		private Vector MultiplyInverseKrrTimesKrcTimes_OLD(Vector vector)
		{
			return MultiplyInverseKrrTimes(MultiplyKrcTimes(vector));
		}

		private (Vector Ua, Vector Fb) CalculateResponse(double[] ub, double[] fa)
		{
			if (SolutionPredictor == null)
			{

				var Ub = Vector.CreateFromArray(ub);
				var Fa = Vector.CreateFromArray(fa);

				Vector KabUb = Kcr.Multiply(Ub, true);
				Vector Ua = inverseKrr.SolveLinearSystem(Fa - KabUb);

				Vector Fb = Kcc * Ub + Kcr.Multiply(Ua, false);

				return (Ua, Fb);
			}
			else
			{
				return SolutionPredictor.CalculateResponse(ub, fa);
			}
		}
		#endregion

		public Vector MultiplyInverseKiiTimes(Vector vector, bool diagonalOnly)
		{
			if (diagonalOnly)
			{
				return inverseKiiDiagonal.Multiply(vector);
			}
			else
			{
				return inverseKii.SolveLinearSystem(vector);
			}
		}

		public Vector MultiplyInverseKrrTimes(Vector vector) => inverseKrr.SolveLinearSystem(vector);

		public Vector MultiplyKbbTimes(Vector vector) => Kbb * vector;

		public Vector MultiplyKbiTimes(Vector vector) => Kbi * vector;

		public Vector MultiplyKccTimes(Vector vector) => Kcc * vector;

		public Vector MultiplyKcrTimes(Vector vector) => Kcr * vector;

		public Vector MultiplyKibTimes(Vector vector) => Kbi.Multiply(vector, true);

		public Vector MultiplyKrcTimes(Vector vector) => Kcr.Multiply(vector, true);

		public void ReorderInternalDofs()
		{
			int[] internalDofs = subdomainDofs.DofsInternalToRemainder;
			(int[] rowIndicesKii, int[] colOffsetsKii) =
				submatrixExtractorBoundaryInternal.ExtractSparsityPattern(Krr, internalDofs);
			(int[] permutation, bool oldToNew) = reordering.FindPermutation(
				internalDofs.Length, rowIndicesKii, colOffsetsKii);

			subdomainDofs.ReorderInternalDofs(DofPermutation.Create(permutation, oldToNew));
		}

		public void ReorderRemainderDofs()
		{
			int[] remainderDofs = subdomainDofs.DofsRemainderToFree;
			SymmetricCscMatrix Kff = linearSystem.Matrix;
			(int[] rowIndicesKrr, int[] colOffsetsKrr) =
				submatrixExtractorCornerRemainder.ExtractSparsityPattern(Kff, remainderDofs);
			(int[] permutation, bool oldToNew) = reordering.FindPermutation(
				remainderDofs.Length, rowIndicesKrr, colOffsetsKrr);

			subdomainDofs.ReorderRemainderDofs(DofPermutation.Create(permutation, oldToNew));
		}

		public class Factory : IFetiDPSubdomainMatrixManagerFactory<SymmetricCscMatrix>
		{
			private readonly bool clearKrrAfterFactorization;

			public Factory(bool clearKrrAfterFactorization = false)
			{
				this.clearKrrAfterFactorization = clearKrrAfterFactorization;
			}

			public ISubdomainMatrixAssembler<SymmetricCscMatrix> CreateAssembler() => new SymmetricCscMatrixAssembler(true);

			public IFetiDPSubdomainMatrixManager CreateMatrixManager(IImplementationProvider provider,
				SubdomainLinearSystem<SymmetricCscMatrix> linearSystem, FetiDPSubdomainDofs subdomainDofs)
			{
				return new FetiDPSubdomainMatrixManagerSymmetricCsc(
					provider, linearSystem, subdomainDofs, clearKrrAfterFactorization);
			}
		}
	}
}
