using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Solution;
using MGroup.Solvers.DofOrdering;

namespace MGroup.Solvers
{
	public interface IDdmSolver : ISolver
	{
		void DistributeNodalLoads(IEnumerable<INodalBoundaryCondition> subdomainLoads, Vector nodalLoadsVector, 
			ISubdomainFreeDofOrdering subdomainDofs);

		void DistributeAllNodalLoads(Vector nodalLoadsVector, ISubdomainFreeDofOrdering subdomainDofs);
	}
}
