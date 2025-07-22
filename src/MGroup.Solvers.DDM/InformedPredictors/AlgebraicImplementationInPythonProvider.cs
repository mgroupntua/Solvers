using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MGroup.LinearAlgebra.Vectors;
using MGroup.Solvers.DDM.MLExtensions.PythonInterop;

namespace MGroup.Solvers.DDM.InformedPredictors
{
	public class AlgebraicImplementationInPythonProvider : IPredictionProvider
	{
		private readonly PythonCaller<InputParams, OutputParams> pythonCaller;
		private readonly int subdomainId;

		public AlgebraicImplementationInPythonProvider(string workDirectory, string pythonInterpreter, string pythonScript,
			int subdomainID)
		{
			this.pythonCaller = new PythonCaller<InputParams, OutputParams>(workDirectory, pythonInterpreter, pythonScript);
			this.subdomainId = subdomainID;
		}

		public (Vector Ua, Vector Fb) CalculateResponse(double[] ub, double[] fa)
		{
			InputParams inputs = new() { SubdomainId = subdomainId, VectorFa = fa, VectorUb = ub };
			(OutputParams outputs, _) = pythonCaller.CallPython(inputs);
			return (Vector.CreateFromArray(outputs.VectorUa), Vector.CreateFromArray(outputs.VectorFb));
		}

		public class InputParams
		{
			public int SubdomainId { get; set; } = -1;

			public double[] VectorFa { get; set; }

			public double[] VectorUb { get; set; }
		}

		public class OutputParams
		{
			public double[] VectorUa { get; set; }

			public double[] VectorFb { get; set; }
		}
	}
}
