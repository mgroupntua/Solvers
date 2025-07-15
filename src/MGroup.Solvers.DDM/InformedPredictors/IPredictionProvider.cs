namespace MGroup.Solvers.DDM.InformedPredictors
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Text;
	using System.Threading.Tasks;

	using MGroup.LinearAlgebra.Vectors;

	public interface IPredictionProvider
	{
		public (Vector Ua, Vector Fb) CalculateResponse(double[] ub, double[] fa);
	}
}
