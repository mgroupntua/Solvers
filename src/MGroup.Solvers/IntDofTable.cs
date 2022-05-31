using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;

//TODO: decide between this and DofTable: Table<INode, IDofType, int> and name the final one as DofTable.
//		DofTable is more expressive and does not require node and dof ids. IntDofTable is more flexible, especially in complex 
//		DDM solvers, and serializable, which is essential for MPI implementations, although converting from/to INode, IDofType
//		is not impossible. Since keys are integers, optimizations concerning storage, retrieval and creation are possible
namespace MGroup.Solvers
{
	/// <summary>
	/// A <see cref="ITable{int, int, int}"/> that associates the freedom degrees of nodes with their ordinal number.
	/// Nodes and dofs are represented by their ids, which must be unique.
	/// Authors: Serafeim Bakalakos
	/// </summary>
	[Serializable]
	public class IntDofTable : ITable<int, int, int>
	{
		private const int defaultInitialCapacity = 1; //There will be at least 1. TODO: perhaps get this from Dictionary class.
		private readonly int initialCapacityForEachDim;
		private readonly Dictionary<int, Dictionary<int, int>> data;

		public IntDofTable(int initialCapacityForEachDim = defaultInitialCapacity)
		{
			this.initialCapacityForEachDim = initialCapacityForEachDim;
			this.data = new Dictionary<int, Dictionary<int, int>>(initialCapacityForEachDim);
		}

		private IntDofTable(Dictionary<int, Dictionary<int, int>> data, int initialCapacityForEachDim = defaultInitialCapacity)
		{
			this.initialCapacityForEachDim = initialCapacityForEachDim;
			this.data = data;
		}

		public int NumEntries //TODO: perhaps this should be cached somehow
		{
			get
			{
				int count = 0;
				foreach (var wholeRow in data) count += wholeRow.Value.Count;
				return count;
			}
		}

		public int EntryCount => throw new NotImplementedException();

		public int this[int row, int col]
		{
			get => data[row][col];

			set
			{
				bool containsRow = data.TryGetValue(row, out Dictionary<int, int> wholeRow);
				if (!containsRow)
				{
					wholeRow = new Dictionary<int, int>(initialCapacityForEachDim);
					data.Add(row, wholeRow);
				}
				wholeRow[col] = value; // This allows changing the value after an entry has been added.
			}
		}

		public void Clear() => data.Clear();

		public bool Contains(int row, int col)
		{
			bool containsRow = data.TryGetValue(row, out Dictionary<int, int> wholeRow);
			if (!containsRow) return false;
			else return wholeRow.ContainsKey(col);
		}

		//TODO: this would be nice to have in Table too.
		public IntDofTable DeepCopy()
		{
			var dataCopy = new Dictionary<int, Dictionary<int, int>>();
			foreach (var wholeRow in this.data)
			{
				// IDof and int are immutable, thus I can just copy the nested dictionary.
				dataCopy.Add(wholeRow.Key, new Dictionary<int, int>(wholeRow.Value));
			}
			return new IntDofTable(dataCopy);
		}

		/// <summary>
		/// Finds and returns the an entry for which the value satisfies <paramref name="predicate"/>.
		/// If none is found, a <see cref="KeyNotFoundException"/> will be thrown.
		/// </summary>
		/// <param name="predicate"></param>
		public (int row, int col, int val) Find(Predicate<int> predicate)
		{
			//TODO: throwing an exception for a common usecase is not the best solution. However returning 
			//      default((int, int, int)) is also bad, since this could be a valid value.
			foreach (var wholeRow in data)
			{
				foreach (var colValPair in wholeRow.Value)
				{
					if (predicate(colValPair.Value)) return (wholeRow.Key, colValPair.Key, colValPair.Value);
				}
			}
			throw new KeyNotFoundException("No entry has the requested value");
		}

		public IEnumerator<(int row, int col, int val)> GetEnumerator()
		{
			foreach (var wholeRow in data)
			{
				foreach (var colValPair in wholeRow.Value)
				{
					yield return (wholeRow.Key, colValPair.Key, colValPair.Value);
				}
			}
		}

		IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

		public IEnumerable<int> GetColumnsOfRow(int row)
		{
			bool containsRow = data.TryGetValue(row, out Dictionary<int, int> wholeRow);
			if (containsRow) return wholeRow.Keys;
			else return Enumerable.Empty<int>();
		}

		public IReadOnlyDictionary<int, int> GetDataOfRow(int row)
		{
			bool exists = data.TryGetValue(row, out Dictionary<int, int> rowData);
			if (!exists) throw new KeyNotFoundException("The provided row is not contained in this table");
			return rowData;
		}

		public IEnumerable<int> GetRows() => data.Keys;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="nodes">If one of these is not contained in the original table, it will be ignored.</param>
		public IntDofTable GetSubtableForNodes(IEnumerable<int> nodes)
		{
			//TODO: I will probably need to control the new ordering, instead of being in the order I meet the dofs
			int dofCounter = 0;
			var subtable = new Dictionary<int, Dictionary<int, int>>();
			foreach (int node in nodes)
			{
				var newNodeData = new Dictionary<int, int>();
				bool hasFreeDofs = this.data.TryGetValue(node, out Dictionary<int, int> oldNodeData);
				if (hasFreeDofs)
				{
					foreach (int dofType in oldNodeData.Keys)
					{
						newNodeData[dofType] = dofCounter++;
					}
					subtable[node] = newNodeData;
				}
			}
			return new IntDofTable(subtable);
		}

		public IEnumerable<int> GetValuesOfRow(int row) //TODO: perhaps I should throw an exception if the row is not found
		{
			bool containsRow = data.TryGetValue(row, out Dictionary<int, int> wholeRow);
			if (containsRow) return wholeRow.Values;
			else return Enumerable.Empty<int>();
		}

		public bool HasSameRowsColumns(IntDofTable other)
		{
			if (this.data.Count != other.data.Count)
			{
				return false;
			}

			foreach (KeyValuePair<int, Dictionary<int, int>> wholeRow in this.data)
			{
				int row = wholeRow.Key;
				bool isCommon = other.data.TryGetValue(row, out Dictionary<int, int> dataOfOtherRow);
				Dictionary<int, int> dataOfThisRow = wholeRow.Value;
				if (isCommon && (dataOfThisRow.Count == dataOfOtherRow.Count))
				{
					foreach (int col in dataOfThisRow.Keys)
					{
						if (!dataOfOtherRow.ContainsKey(col))
						{
							return false;
						}
					}
				}
				else
				{
					return false;
				}
			}

			return true;
		}

		public void ModifyValues(Func<int, int> unaryOperation)
		{
			//TODO: perhaps I should create a new table and replace the existing one once finished.

			foreach (Dictionary<int, int> rowData in data.Values)
			{
				// Create a temporary collection for iterating, while modifying the actual one
				var rowDataAsList = new List<KeyValuePair<int, int>>(rowData);

				foreach (var colValPair in rowDataAsList)
				{
					rowData[colValPair.Key] = unaryOperation(colValPair.Value);
				}
			}
		}

		/// <summary>
		/// Renumbers the dof indices according to the given permutation vector and direction. 
		/// If (<paramref name="oldToNew"/> == true), then newIndex[dof] = <paramref name="permutation"/>[oldIndex[dof]].
		/// Else oldIndex[dof] = <paramref name="permutation"/>[nwIndex[dof]]
		/// </summary>
		/// <param name="permutation">The permutation vector.</param>
		/// <param name="oldToNew">The direction it should be applied to.</param>
		public void Reorder(IReadOnlyList<int> permutation, bool oldToNew)
		{
			IReadOnlyList<int> permutationOldToNew;
			if (oldToNew) permutationOldToNew = permutation;
			else
			{
				//TODO: is it necessary to create a temp oldToNew array?
				var permutationArray = new int[permutation.Count];
				for (int newIdx = 0; newIdx < permutation.Count; ++newIdx) permutationArray[permutation[newIdx]] = newIdx;
				permutationOldToNew = permutationArray;
			}

			foreach (var nodeRow in data.Values)
			{
				var dofIDs = new List<KeyValuePair<int, int>>(nodeRow);
				foreach (var dofIDPair in dofIDs)
				{
					nodeRow[dofIDPair.Key] = permutationOldToNew[dofIDPair.Value];
				}

				// The following throws CollectionModified, although nothing dangerous happens 
				//foreach (var dofID in nodeRow)
				//{
				//    nodeRow[dofID.Key] = permutationOldToNew[dofID.Value];
				//}
			}
		}

		//TODO: use Table.ModifyValues() for this.
		public void ReorderNodeMajor(IReadOnlyList<int> sortedNodes)
		{
			int dofIdx = -1;
			foreach (int node in sortedNodes)
			{
				bool isNodeContained = data.TryGetValue(node, out Dictionary<int, int> dofsOfNode);
				if (isNodeContained)
				{
					// We cannot not update the dictionary during iterating it, thus we copy its Key collection to a list first.
					foreach (int dofType in dofsOfNode.Keys.ToList()) dofsOfNode[dofType] = ++dofIdx;
				}
			}
		}

		public override string ToString()
		{
			StringBuilder builder = new StringBuilder();
			foreach (var nodeData in data.OrderBy(entry => entry.Key))
			{
				builder.AppendLine($"Node {nodeData.Key}:");
				foreach (var dofPair in nodeData.Value)
				{
					builder.Append("\t");
					builder.AppendLine($"Dof id = {dofPair.Key} - index = {dofPair.Value}");
				}
			}
			return builder.ToString();
		}

		/// <summary>
		/// Adds the specified (<paramref name="row"/>, <paramref name="col"/>, <paramref name="value"/>) entry to the table. 
		/// Returns true, if the insertion was successful, or false, if the table already contained the specified entry.
		/// </summary>
		/// <param name="row">The </param>
		/// <param name="col"></param>
		/// <param name="value"></param>
		public bool TryAdd(int row, int col, int value)
		{
			bool containsRow = data.TryGetValue(row, out Dictionary<int, int> wholeRow);
			if (containsRow)
			{
				//return wholeRow.TryAdd(col, value);
				bool colExists = wholeRow.ContainsKey(col);
				if (colExists)
					return false;
				else
				{
					wholeRow.Add(col, value);
					return true;
				}
			}
			else
			{
				wholeRow = new Dictionary<int, int>(initialCapacityForEachDim);
				data.Add(row, wholeRow);
				wholeRow.Add(col, value);
				return true;
			}
		}

		public bool TryGetDataOfRow(int row, out IReadOnlyDictionary<int, int> columnValuePairs)
		{
			bool exists = data.TryGetValue(row, out Dictionary<int, int> rowData);
			columnValuePairs = rowData;
			return exists;
		}

		public bool TryGetValue(int row, int col, out int value)
		{
			bool containsRow = data.TryGetValue(row, out Dictionary<int, int> wholeRow);
			if (!containsRow)
			{
				value = default(int);
				return false;
			}
			else return wholeRow.TryGetValue(col, out value);
		}

		public static (int numDofs, IntDofTable dofOrdering) OrderDofs(DofSet dofSet, Func<int, INode> getNodeFromID)
		{
			var dofTable = new IntDofTable();
			int numDofs = 0;
			foreach (var nodeDofPair in dofSet.EnumerateOrderedNodesDofs())
			{
				dofTable[nodeDofPair.nodeID, nodeDofPair.dofID] = numDofs++;
			}
			return (numDofs, dofTable);
		}

	}
}
