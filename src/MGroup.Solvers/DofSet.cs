using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace MGroup.Solvers
{
	public class DofSet
	{
		//TODO: Perhaps I should use HashSets and order them, when I actually need to number the dofs. 
		private SortedDictionary<int, SortedSet<int>> data = new SortedDictionary<int, SortedSet<int>>();

		public DofSet()
		{
		}

		public static DofSet Deserialize(int[] serializedData)
		{
			var dofSet = new DofSet();
			int i = 0;
			while (i < serializedData.Length)
			{
				int nodeID = serializedData[i];
				Debug.Assert(i + 1 < serializedData.Length, $"Node {nodeID} has no dofs listed.");

				int numDofs = serializedData[i + 1];
				Debug.Assert(i + 1 + numDofs < serializedData.Length, $"Node {nodeID} declared more dofs than they exist.");

				var dofsOfThisNode = new SortedSet<int>();;
				int offset = i + 2;
				for (int j = 0; j < numDofs; ++j)
				{
					dofsOfThisNode.Add(serializedData[offset + j]);
				}
				dofSet.data[nodeID] = dofsOfThisNode;

				i += 2 + numDofs;
			}
			return dofSet;
		}


		public void AddDof(int nodeID, int dofID)
		{
			bool nodeExists = data.TryGetValue(nodeID, out SortedSet<int> dofsOfThisNode);
			if (!nodeExists)
			{
				dofsOfThisNode = new SortedSet<int>();
				data[nodeID] = dofsOfThisNode;
			}
			dofsOfThisNode.Add(dofID);
		}

		public void AddDofs(int nodeID, IEnumerable<int> dofIDs)
		{
			bool nodeExists = data.TryGetValue(nodeID, out SortedSet<int> dofsOfThisNode);
			if (!nodeExists)
			{
				dofsOfThisNode = new SortedSet<int>();
				data[nodeID] = dofsOfThisNode;
			}
			dofsOfThisNode.UnionWith(dofIDs);
		}

		public int Count()
		{
			int numEntries = 0;
			foreach (var nodeDofsPair in data)
			{
				numEntries += nodeDofsPair.Value.Count;
			}
			return numEntries;
		}

		/// <summary>
		/// The order is node major, dof minor.
		/// </summary>
		/// <returns></returns>
		public IEnumerable<(int nodeID, int dofID)> EnumerateOrderedNodesDofs()
		{
			foreach (var nodeDofsPair in data)
			{
				int nodeID = nodeDofsPair.Key;
				foreach (int dofID in nodeDofsPair.Value)
				{
					yield return (nodeID, dofID);
				}
			}
		}

		public IEnumerable<int> GetDofsOfNode(int nodeID) => data[nodeID];

		/// <summary>
		/// Returns a <see cref="DofSet"/> that only contains the dofs this instance has in common with 
		/// <paramref name="other"/>. This intersection excludes nodes that are not common or do not have common dofs in both 
		/// this instance and <paramref name="other"/>. This instance will be unusable afterwards.
		/// </summary>
		/// <param name="other"></param>
		public DofSet IntersectionWith(DofSet other)
		{
			//TODO: Lookups (log(n)) in other.data can be avoided by working with the enumerators of this.data and other.data
			//TODO: Nodes that are not in common or do not have common dofs will be left with empty sets of dofs (int). 
			//		Perhaps they should be cleaned up.
			var result = new DofSet();
			result.data = this.data;
			this.data = null;
			foreach (var nodeDofsPair in result.data)
			{
				int nodeID = nodeDofsPair.Key;
				SortedSet<int> resultDofs = nodeDofsPair.Value;
				bool isNodeCommon = other.data.TryGetValue(nodeID, out SortedSet<int> otherDofs);
				if (isNodeCommon)
				{
					resultDofs.UnionWith(otherDofs);
				}
				else
				{
					resultDofs.Clear();
				}
			}
			return result;
		}

		//TODO: Use byte encodings. For dofs, int is wasteful. Short is better. 
		//		If nodes are the same in both subdomains and sorted, there is noreason to send them too 
		public int[] Serialize() 
		{
			var list = new List<int>();
			foreach (var pair in data)
			{
				int nodeID = pair.Key;
				SortedSet<int> dofIDs = pair.Value;
				list.Add(nodeID);
				list.Add(dofIDs.Count);
				list.AddRange(dofIDs);
			}
			return list.ToArray();
		}


	}
}
