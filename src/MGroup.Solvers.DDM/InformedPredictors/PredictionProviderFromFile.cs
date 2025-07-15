namespace MGroup.Solvers.DDM.InformedPredictors
{
	using System;
	using System.IO;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.LinearAlgebra.Input;

	public class PredictionProviderFromFile : IPredictionProvider
	{
		private readonly Matrix Kcr;
		private readonly Matrix Kcc;
		private readonly Matrix inverseKrr;

		public PredictionProviderFromFile(string path)
		{
			if (!Directory.Exists(path))
			{
				throw new DirectoryNotFoundException($"The directory '{path}' does not exist.");
			}

			// Reader setup
			char[] separators = new char[] { '\n', ' ' };
			var reader = new Array2DReader(false, separators);

			// Load Kcr
			double[,] kcrArray = reader.ReadFile(Path.Combine(path, "Kcr.txt"));
			this.Kcr = Matrix.CreateFromArray(kcrArray);

			// Load Kcc
			double[,] kccArray = reader.ReadFile(Path.Combine(path, "Kcc.txt"));
			this.Kcc = Matrix.CreateFromArray(kccArray);

			// Load Krr and invert it immediately
			double[,] krrArray = reader.ReadFile(Path.Combine(path, "Krr.txt"));
			Matrix Krr = Matrix.CreateFromArray(krrArray);
			this.inverseKrr = Krr.Invert();
		}

		public (Vector Ua, Vector Fb) CalculateResponse(double[] ub, double[] fa)
		{
			var Ub = Vector.CreateFromArray(ub);
			var Fa = Vector.CreateFromArray(fa);

			Vector KabUb = Kcr.Multiply(Ub, true);
			Vector Ua = inverseKrr.Multiply(Fa - KabUb);

			Vector Fb = Kcc * Ub + Kcr.Multiply(Ua, false);

			return (Ua, Fb);
		}
	}
}


