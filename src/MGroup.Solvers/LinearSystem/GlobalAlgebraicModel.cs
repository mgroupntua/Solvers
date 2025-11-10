using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

using MGroup.LinearAlgebra.Exceptions;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Providers;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.Solvers.Assemblers;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.Results;

namespace MGroup.Solvers.LinearSystem
{
	public class GlobalAlgebraicModel<TMatrix> : IAlgebraicModel
		where TMatrix : class, IMatrix
	{
		private readonly ISubdomain subdomain;
		private readonly IModel model;
		private readonly IDofOrderer dofOrderer;
		private readonly ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler;
		private SubdomainVectorAssembler subdomainVectorAssembler;
		private IAlgebraicModelInterpreter boundaryConditionsInterpreter;

		public GlobalAlgebraicModel(IModel model, IDofOrderer dofOrderer,
			ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler)
		{
			this.model = model;
			this.dofOrderer = dofOrderer;
			this.subdomainMatrixAssembler = subdomainMatrixAssembler;
			subdomain = model.EnumerateSubdomains().First();
			LinearSystem = new GlobalLinearSystem<TMatrix>(CheckCompatibleVector, CheckCompatibleMatrix);
			Observers = new HashSet<IAlgebraicModelObserver>();
		}

		public ISubdomainFreeDofOrdering SubdomainFreeDofOrdering { get; private set; }

		IGlobalLinearSystem IAlgebraicModel.LinearSystem => LinearSystem;

		public GlobalLinearSystem<TMatrix> LinearSystem { get; }

		public HashSet<IAlgebraicModelObserver> Observers { get; }

		public int SubdomainID { get; }

		//TODO: Goat - remove the setter and define a solution workflow object that will initialize this at its constructor
		public IAlgebraicModelInterpreter BoundaryConditionsInterpreter 
		{
			get => boundaryConditionsInterpreter;
			set
			{
				boundaryConditionsInterpreter = value;
				subdomainVectorAssembler = new SubdomainVectorAssembler(boundaryConditionsInterpreter.ActiveDofs);
			}
		}

		public void AddToGlobalVector(IVector vector, IElementVectorProvider vectorProvider)
		{
			var globalVector = CheckCompatibleVector(vector);
			var subdomainDofs = SubdomainFreeDofOrdering;
			var elements = model.EnumerateElements(subdomain.ID);
			subdomainVectorAssembler.AddToSubdomainVector(elements, globalVector, vectorProvider, subdomainDofs);
		}

		public void AddToGlobalVector(Func<int, IEnumerable<INodalModelQuantity<IDofType>>> accessLoads, IVector vector)
		{
			var globalVector = CheckCompatibleVector(vector);
			var subdomainDofs = SubdomainFreeDofOrdering;
			var loads = accessLoads(subdomain.ID);
			subdomainVectorAssembler.AddToSubdomainVector(loads, globalVector, subdomainDofs);
		}

		public IMatrix BuildGlobalMatrix(IElementMatrixProvider elementMatrixProvider)
		{
			var subdomainDofs = SubdomainFreeDofOrdering;
			var globalMatrix = subdomainMatrixAssembler.BuildGlobalMatrix(
				subdomainDofs, model.EnumerateElements(subdomain.ID), elementMatrixProvider);
			return globalMatrix;
		}

		public IMatrix CreateEmptyMatrix()
		{
			var subdomainDofs = SubdomainFreeDofOrdering;
			var globalMatrix = subdomainMatrixAssembler.CreateEmptyMatrix(subdomainDofs);
			return globalMatrix;
		}

		IVector IGlobalVectorAssembler.CreateZeroVector() => CreateZeroVector();

		public Vector CreateZeroVector() => Vector.CreateZero(SubdomainFreeDofOrdering.NumFreeDofs);

		public void DoPerElement<TElement>(Action<TElement> elementOperation)
			where TElement: IElementType
		{
			foreach (var element in model.EnumerateElements(subdomain.ID).OfType<TElement>())
			{
				elementOperation(element);
			}
		}

		public NodalResults ExtractAllResults(IVector vector)
		{
			var globalVector = CheckCompatibleVector(vector);
			var results = new Table<int, int, double>();

			// Free dofs
			foreach ((var node, var dof, var freeDofIdx) in SubdomainFreeDofOrdering.FreeDofs)
			{
				results[node, dof] = globalVector[freeDofIdx];
			}

			// Constrained dofs
			var activeDofs = boundaryConditionsInterpreter.ActiveDofs;
			var constraints = model.EnumerateBoundaryConditions(subdomain.ID)
				.SelectMany(x => x.EnumerateNodalBoundaryConditions(model.EnumerateElements(subdomain.ID)))
				.OfType<INodalDirichletBoundaryCondition<IDofType>>();
			foreach (var constraint in constraints)
			{
				results[constraint.Node.ID, activeDofs.GetIdOfDof(constraint.DOF)] = constraint.Amount;
			}

			return new NodalResults(results);
		}

		public double[] ExtractElementVector(IVector vector, IElementType element)
		{
			CheckCompatibleVector(vector);
			var subdomainDofs = SubdomainFreeDofOrdering;
			return subdomainDofs.ExtractVectorElementFromSubdomain(element, vector);
		}

		public double[] ExtractNodalValues(IVector vector, INode node, IDofType[] dofs)
		{
			var globalVector = CheckCompatibleVector(vector);
			var subdomainDofs = SubdomainFreeDofOrdering;
			var nodeConstraints = model.EnumerateBoundaryConditions(subdomain.ID)
				.Select(x => x.EnumerateNodalBoundaryConditions(model.EnumerateElements(subdomain.ID))).OfType<INodalDirichletBoundaryCondition<IDofType>>()
				.Where(x => x.Node.ID == node.ID)
				.ToArray();
			var result = new double[dofs.Length];
			for (var i = 0; i < dofs.Length; ++i)
			{
				var dofID = boundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dofs[i]);
				var dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out var dofIdx);
				if (dofExists)
				{
					result[i] = globalVector[dofIdx];
				}
				else
				{
					var constraint = nodeConstraints.FirstOrDefault(x => x.DOF == dofs[i]);
					if (constraint != null)
					{
						result[i] = constraint.Amount;
					}
					else
					{
						throw new KeyNotFoundException(
							$"The requested {dofs[i]} is neither a free nor a constrained dof of node {node.ID}.");
					}
				}
			}
			return result;
		}

		public double ExtractSingleValue(IVector vector, INode node, IDofType dof)
		{
			var globalVector = CheckCompatibleVector(vector);
			var subdomainDofs = SubdomainFreeDofOrdering;
			var dofID = boundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dof);
			var dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out var dofIdx);
			if (dofExists)
			{
				return globalVector[dofIdx];
			}
			else
			{
				throw new KeyNotFoundException("The requested (node, dof) is not included in the provided vector.");
			}
		}

		protected virtual void OrderDofsInternal()
		{
			SubdomainFreeDofOrdering = dofOrderer.OrderFreeDofs(subdomain, BoundaryConditionsInterpreter);
			foreach (var observer in Observers)
			{
				observer.HandleDofOrderWasModified();
			}
			subdomainMatrixAssembler.HandleDofOrderingWasModified();

			// Define new format and recreate objects using it 
			LinearSystem.Matrix = null;
			LinearSystem.RhsVector = CreateZeroVector();
			LinearSystem.Solution = CreateZeroVector();
		}

		public void OrderDofs()
		{
			model.ConnectDataStructures();
			OrderDofsInternal();
		}

		public virtual void ReorderDofs() => OrderDofsInternal();

		public void RebuildGlobalMatrixPartially(
			IMatrix currentMatrix, Func<int, IEnumerable<IElementType>> accessElements,
			IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate)
		{
			var globalMatrix = CheckCompatibleMatrix(currentMatrix);

			var watch = new Stopwatch();
			watch.Start();

			var subdomainElements = accessElements(subdomain.ID);
			var subdomainMatrix = subdomainMatrixAssembler.RebuildSubdomainMatrix(
				subdomainElements, SubdomainFreeDofOrdering, elementMatrixProvider, predicate);
			if (subdomainMatrix != null)
			{
				//TODO: This is a good point to notify solvers, etc, if the processed matrix is the linear system matrix 
				globalMatrix = subdomainMatrix;
			}
			watch.Stop();
		}

		public IMatrix RebuildGlobalMatrixPartially(IMatrix previousMatrix, 
			Func<int, IEnumerable<IElementType>> accessElements, IElementMatrixProvider elementMatrixProvider)
		{
			// Any change that happened in dofs affected the whole matrix, which needs to be built from scratch.
			return BuildGlobalMatrix(elementMatrixProvider);
		}

		public double[] ReduceSumPerElement<TElement>(int numReducedValues, Func<int, IEnumerable<TElement>> accessElements, 
			Func<TElement, double[]> elementOperation)
			where TElement: IElementType
		{
			var totalResult = new double[numReducedValues];
			foreach (var element in accessElements(subdomain.ID))
			{
				var elementResult = elementOperation(element);
				for (var i = 0; i < numReducedValues; ++i)
				{
					totalResult[i] += elementResult[i];
				}
			}
			return totalResult;
		}

		public double[] ReduceSumPerElement<TElement>(int numReducedValues, Func<int, IEnumerable<TElement>> accessElements,
			Predicate<TElement> isActiveElement, Func<TElement, double[]> elementOperation)
			where TElement: IElementType
		{
			var totalResult = new double[numReducedValues];
			foreach (var element in accessElements(subdomain.ID))
			{
				if (isActiveElement(element))
				{
					var elementResult = elementOperation(element);
					for (var i = 0; i < numReducedValues; ++i)
					{
						totalResult[i] += elementResult[i];
					}
				}
			}
			return totalResult;
		}

		internal TMatrix CheckCompatibleMatrix(IMatrix matrix)
		{
			// Casting inside here is usually safe since all global matrices should be created by this object
			if (matrix is TMatrix casted)
			{
				return casted;
			}

			throw new NonMatchingFormatException("The provided matrix has a different format than the current linear system."
				+ $" Make sure it was created by this linear system object and that the type {typeof(TMatrix)} is used.");
		}

		internal Vector CheckCompatibleVector(IVector vector)
		{
			// Casting inside here is usually safe since all global vectors should be created by this object
			if (vector is Vector casted)
			{
				return casted;
			}

			throw new NonMatchingFormatException("The provided vector has a different format than the current linear system."
				+ $" Make sure it was created by this linear system object.");
		}
	}
}
