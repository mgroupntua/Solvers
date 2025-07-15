using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.Solvers.DDM.MLExtensions.PythonInterop
{
    public class PythonCodeDurationsDto
    {
		/// <summary>
		/// Time spent by Python code on IO operations. In milliseconds.
		/// </summary>
		public long IO { get; set; } = -1;

		/// <summary>
		/// Time spent by Python code on setting up data that would be available if Python run contisuously. In milliseconds.
		/// </summary>
		public long Setup { get; set; } = -1;
	}
}
