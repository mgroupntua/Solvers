using System;
using System.Collections.Generic;
using System.Text;

//TODO: Delete this class and use the corresponding one in LinearAlgebra for the iterative algorithm
namespace MGroup.Solvers.Exceptions
{
	/// <summary>
	/// The exception that is thrown when an iterative solver cannot converge to the desired accuracy. This usually indicates
	/// that the problem is not well defined or that the solver chosen is not appropriate for that problem. However, it is also
	/// possible that the desired accuracy is just too strict, in which case relaxing it will allow the solver to converge to an 
	/// acceptable solution.
	/// </summary>
	public class SolverDidNotConvergeException : Exception
	{
		/// <summary>
		/// Initializes a new instance of the <see cref="SolverDidNotConvergeException"/> class.
		/// </summary>
		public SolverDidNotConvergeException()
		{ }

		/// <summary>
		/// Initializes a new instance of the <see cref="SolverDidNotConvergeException"/> class with a specified error message.
		/// </summary>
		/// <param name="message">The error message that explains the reason for the exception.</param>
		public SolverDidNotConvergeException(string message) : base(message)
		{ }

		/// <summary>
		/// Initializes a new instance of the <see cref="SolverDidNotConvergeException"/> class with a specified error message 
		/// and a reference to the inner exception that is the cause of this exception.
		/// </summary>
		/// <param name="message">The error message that explains the reason for the exception.</param>
		/// <param name="inner">The exception that is the cause of the current exception. If the innerException parameter is not 
		///     a null reference, the current exception is raised in a catch block that handles the inner exception. </param>
		public SolverDidNotConvergeException(string message, Exception inner) : base(message, inner)
		{ }
	}
}
