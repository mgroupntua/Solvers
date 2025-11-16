namespace MGroup.Solvers.DDM.PSM.StiffnessMatrices
{
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;

	public interface IPsmSubdomainMatrixManager
	{
		bool IsEmpty { get; }

		IReadOnlyMatrix CalcSchurComplement();

		void ClearSubMatrices();

		void ExtractKiiKbbKib();

		void HandleDofsWereModified();

		void InvertKii();

		Vector MultiplyInverseKii(Vector vector);

		Vector MultiplyKbb(Vector vector);

		Vector MultiplyKbi(Vector vector);

		Vector MultiplyKib(Vector vector);

		/// <summary>
		/// S[s] * x = (Kbb[s] - Kbi[s] * inv(Kii[s]) * Kib[s]) * x, where s is a subdomain and x is the <paramref name="input"/>.
		/// </summary>
		/// <param name="input">The displacements that correspond to boundary dofs of this subdomain.</param>
		/// <param name="output">The forces that correspond to boundary dofs of this subdomain.</param>
		public void MultiplySchurComplementImplicitly(Vector input, Vector output)
		{
			output.CopyFrom(MultiplyKbb(input));
			Vector temp = MultiplyKib(input);
			temp = MultiplyInverseKii(temp);
			temp = MultiplyKbi(temp);
			output.SubtractIntoThis(temp);
		}

		void ReorderInternalDofs();
	}
}
