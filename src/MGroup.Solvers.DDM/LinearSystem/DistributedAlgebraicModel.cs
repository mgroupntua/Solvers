//TODO: Move all Extract~() methods to a dedicated DistributedValueExtractor class.
namespace MGroup.Solvers.DDM.LinearSystem
{
	using System;
	using System.Collections.Concurrent;
	using System.Collections.Generic;
	using System.Diagnostics;
	using System.Linq;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
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
	using MGroup.Solvers.DDM.DiscretizationExtensions;
	using MGroup.Solvers.DofOrdering;
	using MGroup.Solvers.Results;

	public class DistributedAlgebraicModel<TMatrix> : IAlgebraicModel
		where TMatrix : class, IMatrix
	{
		private IAlgebraicModelInterpreter boundaryConditionsInterpreter;
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly IDofOrderer dofOrderer;
		private readonly ReanalysisOptions reanalysis;
		private readonly Dictionary<int, ISubdomainMatrixAssembler<TMatrix>> subdomainMatrixAssemblers;
		private SubdomainVectorAssembler subdomainVectorAssembler;

		public DistributedAlgebraicModel(IComputeEnvironment environment, IModel model, IDofOrderer dofOrderer,
			ISubdomainTopology subdomainTopology, ISubdomainMatrixAssembler<TMatrix> subdomainMatrixAssembler,
			ReanalysisOptions reanalysisOptions)
		{
			this.environment = environment;
			this.model = model;
			this.dofOrderer = dofOrderer;

			this.subdomainMatrixAssemblers = environment.CalcNodeData(subdomainID => subdomainMatrixAssembler.Clone());

			this.LinearSystem = new DistributedLinearSystem<TMatrix>(CheckCompatibleVector, CheckCompatibleMatrix);
			this.SubdomainLinearSystems = environment.CalcNodeData(
				subdomainID => new SubdomainLinearSystem<TMatrix>(this, subdomainID));

			this.SubdomainTopology = subdomainTopology;
			this.reanalysis = reanalysisOptions;
			this.SubdomainTopology.Initialize(environment, model, s => SubdomainFreeDofOrderings[s]); 
			this.SubdomainTopology.FindCommonNodesBetweenSubdomains(); //TODO: what about problems where the mesh is repartitioned in some iterations?

			Observers = new HashSet<IAlgebraicModelObserver>();
		}

		//TODO: Goat - remove the setter and define a solution workflow object that will initialize this at its constructor
		public IAlgebraicModelInterpreter BoundaryConditionsInterpreter
		{
			get => boundaryConditionsInterpreter;
			set
			{
				boundaryConditionsInterpreter = value;
				this.subdomainVectorAssembler = new SubdomainVectorAssembler(boundaryConditionsInterpreter.ActiveDofs);
			}
		}

		public ConcurrentDictionary<int, ISubdomainFreeDofOrdering> SubdomainFreeDofOrderings { get; }
			= new ConcurrentDictionary<int, ISubdomainFreeDofOrdering>();

		//TODOMPI: Perhaps this and subdomain free dof orderings must be managed by a dedicated component. That component could 
		//		also perform some of all these tasks done by SubdomainTopology.
		public DistributedOverlappingIndexer FreeDofIndexer { get; private set; }

		IGlobalLinearSystem IAlgebraicModel.LinearSystem => LinearSystem;

		public DistributedLinearSystem<TMatrix> LinearSystem { get; }

		public HashSet<IAlgebraicModelObserver> Observers { get; }

		public Dictionary<int, SubdomainLinearSystem<TMatrix>> SubdomainLinearSystems { get; }

		public ISubdomainTopology SubdomainTopology { get; }

		public void AddToGlobalVector(IVector vector, IElementVectorProvider vectorProvider)
		{
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				IEnumerable<IElementType> elements = model.EnumerateElements(subdomainID);
				var subdomainVector = distributedVector.LocalVectors[subdomainID];
				subdomainVectorAssembler.AddToSubdomainVector(elements, subdomainVector, vectorProvider, subdomainDofs);
			});

			// Element loads at the same boundary dof must be summed across subdomains. 
			// This way the resulting global vector is the same as the corresponding global vector without domain decomposition.
			distributedVector.SumOverlappingEntries();
		}

		public void AddToGlobalVector(Func<int, IEnumerable<INodalModelQuantity<IDofType>>> accessLoads, IVector vector)
		{
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				IEnumerable<INodalModelQuantity<IDofType>> loads = accessLoads(subdomainID);
				var subdomainVector = distributedVector.LocalVectors[subdomainID];
				subdomainVectorAssembler.AddToSubdomainVector(loads, subdomainVector, subdomainDofs);
			});

			// Nodal loads at the same boundary dof are the same across all relevant subdomains, 
			// so we do not need to sum overlapping entries
		}

		public IMatrix BuildGlobalMatrix(IElementMatrixProvider elementMatrixProvider)
		{
			var globalMatrix = new DistributedOverlappingMatrix<TMatrix>(FreeDofIndexer);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				TMatrix matrix = subdomainMatrixAssemblers[subdomainID].BuildGlobalMatrix(
						subdomainDofs, model.EnumerateElements(subdomainID), elementMatrixProvider);
				globalMatrix.LocalMatrices[subdomainID] = matrix;
			});
			return globalMatrix;
		}

		public IMatrix CreateEmptyMatrix()
		{
			var globalMatrix = new DistributedOverlappingMatrix<TMatrix>(FreeDofIndexer);
			environment.DoPerNode(subdomainID =>
			{
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
				TMatrix matrix = subdomainMatrixAssemblers[subdomainID].CreateEmptyMatrix(subdomainDofs);
				globalMatrix.LocalMatrices[subdomainID] = matrix;
			});
			return globalMatrix;
		}

		IVector IGlobalVectorAssembler.CreateZeroVector() => CreateZeroVector();

		public DistributedOverlappingVector CreateZeroVector() => new DistributedOverlappingVector(FreeDofIndexer);

		public void DoPerElement<TElement>(Action<TElement> elementOperation)
			where TElement: IElementType
		{
			environment.DoPerNode(subdomainID =>
			{
				foreach (TElement element in model.EnumerateElements(subdomainID))
				{
					elementOperation(element);
				}
			});
		}

		public NodalResults ExtractAllResults(int subdomainID, IVector vector)
		{
			var results = new Table<int, int, double>();

			// Free dofs
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			Vector subdomainVector = distributedVector.LocalVectors[subdomainID];
			ISubdomainFreeDofOrdering subdomainFreeDofs = SubdomainFreeDofOrderings[subdomainID];
			foreach ((int node, int dof, int freeDofIdx) in subdomainFreeDofs.FreeDofs)
			{
				results[node, dof] = subdomainVector[freeDofIdx];
			}

			// Constrained dofs
			ActiveDofs activeDofs = boundaryConditionsInterpreter.ActiveDofs;
			var constraints = model.FindDirichletBCsOfSubdomain(subdomainID);
			foreach (var constraint in constraints)
			{
				results[constraint.Node.ID, activeDofs.GetIdOfDof(constraint.DOF)] = constraint.Amount;
			}

			return new NodalResults(results);
		}

		public double[] ExtractElementVector(IVector vector, IElementType element)
		{
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);

			int s = element.SubdomainID;
			bool isLocalElement = SubdomainFreeDofOrderings.TryGetValue(s, out ISubdomainFreeDofOrdering subdomainDofs);
			if (!isLocalElement)
			{
				throw new ArgumentException($"Element {element.ID} belongs to subdomain {s}, which is not local to this " +
					$"memory space and thus all needed data are unavailable");
			}

			return subdomainDofs.ExtractVectorElementFromSubdomain(element, distributedVector.LocalVectors[s]);
		}

		public NodalResults ExtractGlobalResults(IVector vector, double differentValueTolerance)
		{
			if (!(environment is SequentialSharedEnvironment) && !(environment is TplSharedEnvironment))
			{
				throw new NotImplementedException();
			}
			var globalResults = new NodalResults(new Table<int, int, double>());
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				NodalResults subdomainResults = ExtractAllResults(subdomain.ID, vector);
				globalResults.UnionWith(subdomainResults, differentValueTolerance);
			}
			return globalResults;
		}

		public double[] ExtractNodalValues(IVector vector, INode node, IDofType[] dofs)
		{
			if (!(environment is SequentialSharedEnvironment) && !(environment is TplSharedEnvironment))
			{
				throw new NotImplementedException("We need to locate the correct subdomain first");
			}
			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			if (node.Subdomains.Count == 1) // Internal nodes are straightforward
			{
				int s = node.Subdomains.First();
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[s];
				INodalDirichletBoundaryCondition<IDofType>[] nodeConstraints = model.FindDirichletBCsOfNode(node, s);
				var result = new double[dofs.Length];
				for (int i = 0; i < dofs.Length; ++i)
				{
					int dofID = boundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dofs[i]);
					bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out int dofIdx);
					if (dofExists)
					{
						result[i] = distributedVector.LocalVectors[s][dofIdx];
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
			else // Boundary nodes are tricky
			{
				throw new NotImplementedException();
			}
		}

		public double ExtractSingleValue(IVector vector, INode node, IDofType dof)
		{
			if (!(environment is SequentialSharedEnvironment) && !(environment is TplSharedEnvironment))
			{
				throw new NotImplementedException("We need to locate the correct subdomain first");
			}

			DistributedOverlappingVector distributedVector = CheckCompatibleVector(vector);
			if (node.Subdomains.Count == 1) // Internal nodes are straightforward
			{
				int s = node.Subdomains.First();
				ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[s];
				int dofID = BoundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dof);
				bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out int dofIdx);
				if (dofExists)
				{
					return distributedVector.LocalVectors[s][dofIdx];
				}
				else
				{
					throw new KeyNotFoundException("The requested (node, dof) is not included in the provided vector.");
				}
			}
			else // Boundary nodes are tricky
			{
				//TODO: this should be a parameter of the dedicated object that extracts values. It could also be an input of
				//		this method, but that is not a good idea, since it assumes the caller knows about subdomains 
				double tol = 1E-6; 
				var comparer = new ValueComparer(tol);

				var values = new List<double>();
				foreach (int s in node.Subdomains)
				{
					ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[s];
					int dofID = BoundaryConditionsInterpreter.ActiveDofs.GetIdOfDof(dof);
					bool dofExists = subdomainDofs.FreeDofs.TryGetValue(node.ID, dofID, out int dofIdx);
					if (dofExists)
					{
						// It is possible that this dof is activated by the elements of only 1 subdomain, even if the node
						// belongs to more than 1 subdomains
						values.Add(distributedVector.LocalVectors[s][dofIdx]);
					}
				}

				if (values.Count == 0)
				{
					// It is also possible that the client is stupid
					throw new KeyNotFoundException("The requested (node, dof) is not included in the provided vector.");
				}
				else
				{
					double sum = values[0];
					for (int i = 1; i < values.Count; ++i)
					{
						if (comparer.AreEqual(values[i - 1], values[i]))
						{
							sum += values[i];
						}
						else
						{
							throw new NotImplementedException("There are multiple different values of for this (node, dof) pair");
						}
					}

					return sum / values.Count;
				}
			}
		}

		public void OrderDofs()
		{
			model.ConnectDataStructures();

			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				SubdomainFreeDofOrderings[subdomainID] = dofOrderer.OrderFreeDofs(subdomain, BoundaryConditionsInterpreter);
				subdomainMatrixAssemblers[subdomainID].HandleDofOrderingWasModified();
			});

			SubdomainTopology.FindCommonDofsBetweenSubdomains();
			FreeDofIndexer = SubdomainTopology.CreateDistributedVectorIndexer(s => SubdomainFreeDofOrderings[s].FreeDofs);

			foreach (IAlgebraicModelObserver observer in Observers)
			{
				observer.HandleDofOrderWasModified();
			}

			// Define new format and recreate objects using it 
			LinearSystem.Matrix = null; // If this is set to null, I cannot reuse portions of the previous matrix
			LinearSystem.RhsVector = CreateZeroVector();
			LinearSystem.Solution = CreateZeroVector();
		}

		public void ReorderDofs()
		{
			environment.DoPerNode(subdomainID =>
			{
				if (!reanalysis.SubdomainFreeDofs || reanalysis.ModifiedSubdomains.IsConnectivityModified(subdomainID))
				{
					#region log
					//Debug.WriteLine($"Ordering dofs of subdomain {subdomainID}");
					//Console.WriteLine($"Ordering dofs of subdomain {subdomainID}");
					#endregion

					ISubdomain subdomain = model.GetSubdomain(subdomainID);
					SubdomainFreeDofOrderings[subdomainID] = dofOrderer.OrderFreeDofs(subdomain, BoundaryConditionsInterpreter);
					subdomainMatrixAssemblers[subdomainID].HandleDofOrderingWasModified();
				}
				else
				{
					Debug.Assert(SubdomainFreeDofOrderings.ContainsKey(subdomainID));
					return;
				}
			});

			if (reanalysis.IntersubdomainFreeDofs)
			{
				SubdomainTopology.RefindCommonDofsBetweenSubdomains(s => reanalysis.ModifiedSubdomains.IsConnectivityModified(s));
				FreeDofIndexer = SubdomainTopology.RecreateDistributedVectorIndexer(s => SubdomainFreeDofOrderings[s].FreeDofs,
					FreeDofIndexer, s => reanalysis.ModifiedSubdomains.IsConnectivityModified(s));
			}
			else
			{
				SubdomainTopology.FindCommonDofsBetweenSubdomains();
				FreeDofIndexer = SubdomainTopology.CreateDistributedVectorIndexer(s => SubdomainFreeDofOrderings[s].FreeDofs);
			}

			foreach (IAlgebraicModelObserver observer in Observers)
			{
				observer.HandleDofOrderWasModified();
			}

			// Define new format and recreate objects using it 
			//LinearSystem.Matrix = null; // If this is set to null, I cannot reuse portions of the previous matrix
			LinearSystem.RhsVector = CreateZeroVector();
			LinearSystem.Solution = CreateZeroVector();
		}

		public void RebuildGlobalMatrixPartially(IMatrix currentMatrix, Func<int, IEnumerable<IElementType>> accessElements, 
			IElementMatrixProvider elementMatrixProvider, IElementMatrixPredicate predicate)
		{
			DistributedOverlappingMatrix<TMatrix> distributedMatrix = CheckCompatibleMatrix(currentMatrix);
			environment.DoPerNode(subdomainID =>
			{
				IEnumerable<IElementType> subdomainElements = accessElements(subdomainID);
				TMatrix subdomainMatrix = subdomainMatrixAssemblers[subdomainID].RebuildSubdomainMatrix(
					subdomainElements, SubdomainFreeDofOrderings[subdomainID], elementMatrixProvider, predicate);
				if (subdomainMatrix != null)
				{
					//TODO: This is a good point to notify solvers, etc, if the processed matrix is the linear system matrix 
					distributedMatrix.LocalMatrices[subdomainID] = subdomainMatrix;
				}
			});
		}

		public IMatrix RebuildGlobalMatrixPartially(IMatrix previousMatrix, 
			Func<int, IEnumerable<IElementType>> accessElements, IElementMatrixProvider elementMatrixProvider)
		{
			var previousMatrixDistributed = (DistributedOverlappingMatrix<TMatrix>)previousMatrix;
			var newGlobalMatrix = new DistributedOverlappingMatrix<TMatrix>(FreeDofIndexer);
			environment.DoPerNode(subdomainID =>
			{
				if (!reanalysis.SubdomainMatrix || reanalysis.ModifiedSubdomains.IsMatrixModified(subdomainID))
				{
					#region log
					//Debug.WriteLine($"Building Kff of subdomain {subdomainID}");
					//Console.WriteLine($"Building Kff of subdomain {subdomainID}");
					#endregion

					ISubdomainFreeDofOrdering subdomainDofs = SubdomainFreeDofOrderings[subdomainID];
					TMatrix matrix = subdomainMatrixAssemblers[subdomainID].BuildGlobalMatrix(
							subdomainDofs, accessElements(subdomainID), elementMatrixProvider);
					newGlobalMatrix.LocalMatrices[subdomainID] = matrix;
				}
				else
				{
					//TODO: check the previous matrix somehow
					newGlobalMatrix.LocalMatrices[subdomainID] = previousMatrixDistributed.LocalMatrices[subdomainID];
				}
			});

			return newGlobalMatrix;
		}

		public double[] ReduceSumPerElement<TElement>(int numReducedValues, Func<int, IEnumerable<TElement>> accessElements,
			Func<TElement, double[]> elementOperation)
			where TElement: IElementType
		{
			Dictionary<int, double[]> subdomainResults = environment.CalcNodeData(subdomainID =>
			{
				var subdomainResult = new double[numReducedValues];
				foreach (TElement element in accessElements(subdomainID))
				{
					double[] elementResult = elementOperation(element);
					for (int i = 0; i < numReducedValues; ++i)
					{
						subdomainResult[i] += elementResult[i];
					}
				}
				return subdomainResult;
			});

			double[] totalResult = environment.AllReduceSum(numReducedValues, subdomainResults);
			return totalResult;
		}

		public double[] ReduceSumPerElement<TElement>(int numReducedValues, Func<int, IEnumerable<TElement>> accessElements,
			Predicate<TElement> isActiveElement, Func<TElement, double[]> elementOperation)
			where TElement: IElementType
		{
			Dictionary<int, double[]> subdomainResults = environment.CalcNodeData(subdomainID =>
			{
				var subdomainResult = new double[numReducedValues];
				foreach (TElement element in accessElements(subdomainID))
				{
					if (isActiveElement(element))
					{
						double[] elementResult = elementOperation(element);
						for (int i = 0; i < numReducedValues; ++i)
						{
							subdomainResult[i] += elementResult[i];
						}
					}
				}
				return subdomainResult;
			});

			double[] totalResult = environment.AllReduceSum(numReducedValues, subdomainResults);
			return totalResult;
		}

		internal DistributedOverlappingMatrix<TMatrix> CheckCompatibleMatrix(IMatrix matrix)
		{
			if (matrix is DistributedOverlappingMatrix<TMatrix> distributed)
			{
				if (FreeDofIndexer.IsCompatibleWith(distributed.Indexer))
				{
					return distributed;
				}
			}

			throw new NonMatchingFormatException(
				"The provided matrix has a different format than the current distributed linear system."
				+ $" Ensure it was created by this linear system object and that the type {typeof(TMatrix)} is used.");
		}

		internal DistributedOverlappingVector CheckCompatibleVector(IVector vector)
		{
			if (vector is DistributedOverlappingVector distributed)
			{
				if (FreeDofIndexer.IsCompatibleWith(distributed.Indexer))
				{
					return distributed;
				}
			}

			throw new NonMatchingFormatException(
				"The provided vector has a different format than the current distributed linear system."
				+ $" Ensure it was created by this linear system object.");
		}
	}
}
