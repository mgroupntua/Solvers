//using System;
//using System.Collections.Generic;
//using System.Text;
//using MGroup.LinearAlgebra.Matrices;
//using MGroup.LinearAlgebra.Vectors;
//using MGroup.MSolve.DataStructures;
//using MGroup.MSolve.Discretization;
//using MGroup.MSolve.Discretization.BoundaryConditions;
//using MGroup.MSolve.Solution;
//using MGroup.MSolve.Solution.AlgebraicModel;


//using MGroup.MSolve.Solution.LinearSystem;
//using MGroup.Solvers.Assemblers;
//using MGroup.Solvers.DofOrdering;
//using MGroup.Solvers.DofOrdering.Reordering;
//using MGroup.Solvers.LinearSystem;

//namespace MGroup.Solvers
//{
//	public class MockDdmSolver : IDdmSolver
//	{
//		private readonly IModel model;
//		private readonly IAlgebraicModel algebraicModel;

//		public MockDdmSolver(IModel model, IAlgebraicModel algebraicModel)
//		{
//			this.model = model;
//			this.algebraicModel = algebraicModel;
//			var dofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
//			var subdomainMatrixAssembler = new DenseMatrixAssembler();
//			this.LinearSystem = algebraicModel.LinearSystem;
//			LinearSystem.Observers.Add(this);
//		}

//		public IGlobalLinearSystem LinearSystem { get; }

//		public ISolverLogger Logger => throw new NotImplementedException();

//		public string Name => throw new NotImplementedException();

//		public void DistributeNodalLoads(IEnumerable<INodalBoundaryCondition> subdomainLoads, Vector nodalLoadsVector, 
//			ISubdomainFreeDofOrdering subdomainDofs)
//		{
//			throw new NotImplementedException("This should probably be done privately by the solver without affecting global vectors used by other components");
//			DofTable freeDofs = subdomainDofs.FreeDofs;
//			foreach (INodalBoundaryCondition load in subdomainLoads)
//			{
//				int freeDofIdx = freeDofs[load.Node, load.DOF];
//				nodalLoadsVector[freeDofIdx] /= load.Node.Subdomains.Count;
//			}
//		}

//		public void DistributeAllNodalLoads(Vector nodalLoadsVector, ISubdomainFreeDofOrdering subdomainDofs)
//		{
//			throw new NotImplementedException("This should probably be done privately by the solver without affecting global vectors used by other components");
//			DofTable freeDofs = subdomainDofs.FreeDofs;
//			foreach ((INode node, _, int freeDofIdx) in freeDofs)
//			{
//				nodalLoadsVector[freeDofIdx] /= node.Subdomains.Count;
//			}
//		}

//		public void HandleMatrixWillBeSet()
//		{
//			throw new NotImplementedException();
//		}

//		public void Initialize()
//		{
//			throw new NotImplementedException();
//		}

//		public void PreventFromOverwrittingSystemMatrices()
//		{
//			throw new NotImplementedException();
//		}

//		public void Solve()
//		{
//			throw new NotImplementedException();
//		}
//	}
//}
