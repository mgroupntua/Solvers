//TODO: Perhaps the common nodes and common dofs should be calculated and handled by different classes. Common dofs must be created
//		immediately after ordering subdomain free dofs. Common nodes, immediately after partitioning/repartitioning.
//TODO: Another way to make this class smaller is to delegate local subdomain operations to a dedicated class, while this one 
//		handles the communications.
//TODO: There is some duplication between processing nodes and processing dofs.
namespace MGroup.Solvers.DDM
{
	using System;
	using System.Collections.Concurrent;
	using System.Diagnostics;

	using MGroup.Environments;
	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers.DofOrdering;

	/// <remarks>
	/// In the current design, the subdomain neighbors and their common (boundary) nodes are supposed to remain constant 
	/// throughout the analysis. The dofs at these nodes may be different per subdomain and even change during the analysis. 
	/// </remarks>
	public class SubdomainTopologyGeneral : ISubdomainTopology
	{
		/// <summary>
		/// First key: local subdomain. Second key: neighbor subdomain. Value: dofs at common nodes between these 2 subdomains.
		/// Again there is duplication between the 2 subdomains of each pair.
		/// </summary>
		protected readonly ConcurrentDictionary<int, Dictionary<int, DofSet>> commonDofsBetweenSubdomains
			= new ConcurrentDictionary<int, Dictionary<int, DofSet>>();

		/// <summary>
		/// The common nodes for two neighbors s0, s1 will be found and stored twice: once for s0 and once for s1.
		/// </summary>
		private readonly ConcurrentDictionary<int, Dictionary<int, SortedSet<int>>> commonNodesWithNeighborsPerSubdomain
			= new ConcurrentDictionary<int, Dictionary<int, SortedSet<int>>>();

		protected IComputeEnvironment environment;
		private IModel model;
		private Func<int, ISubdomainFreeDofOrdering> getSubdomainFreeDofs;
		private Dictionary<int, SortedSet<int>> neighborsPerSubdomain;

		public SubdomainTopologyGeneral()
		{
		}

		public DistributedOverlappingIndexer CreateDistributedVectorIndexer(Func<int, IntDofTable> getSubdomainDofs)
		{
			var indexer = new DistributedOverlappingIndexer(environment);
			indexer.Initialize(subdomainID => InitializeIndexer(subdomainID, getSubdomainDofs));
			return indexer;
		}

		public virtual void FindCommonDofsBetweenSubdomains()
		{
			// Find all dofs of each subdomain at the common nodes.
			environment.DoPerNode(subdomainID =>
			{
				Dictionary<int, DofSet> commonDofs = FindLocalSubdomainDofsAtCommonNodes(subdomainID);
				commonDofsBetweenSubdomains[subdomainID] = commonDofs;
			});

			// Send these dofs to the corresponding neighbors and receive theirs.
			Dictionary<int, AllToAllNodeData<int>> transferDataPerSubdomain = environment.CalcNodeData(subdomainID =>
			{
				var transferData = new AllToAllNodeData<int>();
				transferData.sendValues = new ConcurrentDictionary<int, int[]>();
				foreach (int neighborID in GetNeighborsOfSubdomain(subdomainID))
				{
					DofSet commonDofs = commonDofsBetweenSubdomains[subdomainID][neighborID];

					//TODOMPI: Serialization & deserialization should be done by the environment, if necessary.
					transferData.sendValues[neighborID] = commonDofs.Serialize();
				}

				// No buffers for receive values yet, since their lengths are unknown. 
				// Let the environment create them, by using extra communication.
				transferData.recvValues = new ConcurrentDictionary<int, int[]>();
				return transferData;
			});
			environment.NeighborhoodAllToAll(transferDataPerSubdomain, false);

			// Find the intersection between the dofs of a subdomain and the ones received by its neighbor.
			environment.DoPerNode(subdomainID =>
			{
				AllToAllNodeData<int> transferData = transferDataPerSubdomain[subdomainID];
				foreach (int neighborID in GetNeighborsOfSubdomain(subdomainID))
				{
					DofSet receivedDofs = DofSet.Deserialize(transferData.recvValues[neighborID]);
					commonDofsBetweenSubdomains[subdomainID][neighborID] =
						commonDofsBetweenSubdomains[subdomainID][neighborID].IntersectionWith(receivedDofs);
				}
			});
		}

		public void FindCommonNodesBetweenSubdomains()
		{
			environment.DoPerNode(subdomainID =>
			{
				ISubdomain subdomain = model.GetSubdomain(subdomainID);
				var commonNodesOfThisSubdomain = new Dictionary<int, SortedSet<int>>();
				foreach (INode node in subdomain.EnumerateNodes())
				{
					if (node.Subdomains.Count == 1) continue; // internal node

					foreach (int otherSubdomainID in node.Subdomains)
					{
						if (otherSubdomainID == subdomainID) continue; // one of all will be the current subdomain

						Debug.Assert(neighborsPerSubdomain[subdomainID].Contains(otherSubdomainID),
							$"Subdomain {otherSubdomainID} is not listed as a neighbor of subdomain {subdomainID}," +
							$" but node {node.ID} exists in both subdomains");

						bool subdomainPairExists = commonNodesOfThisSubdomain.TryGetValue(
							otherSubdomainID, out SortedSet<int> commonNodes);
						if (!subdomainPairExists)
						{
							commonNodes = new SortedSet<int>();
							commonNodesOfThisSubdomain[otherSubdomainID] = commonNodes;
						}
						commonNodes.Add(node.ID);
					}
				}
				this.commonNodesWithNeighborsPerSubdomain[subdomainID] = commonNodesOfThisSubdomain;
			});
		}

		//TODOMPI: this is not very safe. It is easy to mix up the two subdomains, which will lead to NullReferenceException if
		//      they belong to different clusters/MPI processes. Perhaps this info should be given together with neighborsPerSubdomain
		//      to avoid such cases. Or ISubdomain could contain both of these data.
		public DofSet GetCommonDofsOfSubdomains(int localSubdomainID, int neighborSubdomainID)
			=> commonDofsBetweenSubdomains[localSubdomainID][neighborSubdomainID];

		//TODOMPI: this is not very safe. It is easy to mix up the two subdomains, which will lead to NullReferenceException if
		//      they belong to different clusters/MPI processes. Perhaps this info should be given together with neighborsPerSubdomain
		//      to avoid such cases. Or ISubdomain could contain both of these data.
		/// <summary>
		/// Find the nodes of subdomain <paramref name="localSubdomainID"/> that are common with subdomain 
		/// <paramref name="neighborSubdomainID"/>. The order of these two subdomains is important.
		/// </summary>
		/// <param name="localSubdomainID">
		/// The main subdomain being processed by the current execution unit.
		/// </param>
		/// <param name="neighborSubdomainID">A neighboring subdomain of <paramref name="localSubdomainID"/>.</param>
		public SortedSet<int> GetCommonNodesOfSubdomains(int localSubdomainID, int neighborSubdomainID)
			=> commonNodesWithNeighborsPerSubdomain[localSubdomainID][neighborSubdomainID];

		public SortedSet<int> GetNeighborsOfSubdomain(int subdomainID) => neighborsPerSubdomain[subdomainID];

		public void Initialize(IComputeEnvironment environment, IModel model, 
			Func<int, ISubdomainFreeDofOrdering> getSubdomainFreeDofs)
		{
			this.environment = environment;
			this.model = model;
			this.getSubdomainFreeDofs = getSubdomainFreeDofs;
			this.neighborsPerSubdomain = environment.CalcNodeData(subdomainID =>
			{
				ComputeNode computeNode = environment.GetComputeNode(subdomainID);
				var neighbors = new SortedSet<int>();
				neighbors.UnionWith(computeNode.Neighbors);
				return neighbors;
			});
		}

		public DistributedOverlappingIndexer RecreateDistributedVectorIndexer(Func<int, IntDofTable> getSubdomainDofs,
			DistributedOverlappingIndexer previousIndexer, Func<int, bool> isModifiedSubdomain)
		{
			DistributedOverlappingIndexer newIndexer = previousIndexer.ReuseAsBasisForNewIndexer(subdomainID =>
			{
				if (isModifiedSubdomain(subdomainID))
				{
					return InitializeIndexer(subdomainID, getSubdomainDofs);
				}
				else
				{
					return LocalIndexerDto.CreateUnmodified();
				}
			});

			return newIndexer;
		}

		public virtual void RefindCommonDofsBetweenSubdomains(Func<int, bool> isModifiedSubdomain)
		{
			FindCommonDofsBetweenSubdomains();
		}

		protected Dictionary<int, DofSet> FindLocalSubdomainDofsAtCommonNodes(int subdomainID)
		{
			var commonDofsOfSubdomain = new Dictionary<int, DofSet>();
			IntDofTable freeDofs = getSubdomainFreeDofs(subdomainID).FreeDofs;
			foreach (int neighborID in GetNeighborsOfSubdomain(subdomainID))
			{
				var dofSet = new DofSet();
				foreach (int nodeID in GetCommonNodesOfSubdomains(subdomainID, neighborID))
				{
					dofSet.AddDofs(nodeID, freeDofs.GetColumnsOfRow(nodeID));
				}
				commonDofsOfSubdomain[neighborID] = dofSet;
			}
			return commonDofsOfSubdomain;
		}

		private LocalIndexerDto InitializeIndexer(int subdomainID, Func<int, IntDofTable> getSubdomainDofs)
		{
			IntDofTable subdomainDofs = getSubdomainDofs(subdomainID);

			var allCommonDofIndices = new Dictionary<int, int[]>();
			foreach (int neighborID in GetNeighborsOfSubdomain(subdomainID))
			{
				DofSet commonDofs = GetCommonDofsOfSubdomains(subdomainID, neighborID);
				var commonDofIndices = new List<int>(commonDofs.Count());
				foreach ((int nodeID, int dofID) in commonDofs.EnumerateOrderedNodesDofs())
				{
					//TODO: It would be faster to iterate each node and then its dofs. Same for DofTable. 
					//		Even better let DofTable take DofSet as argument and return the indices.
					bool isRelevant = subdomainDofs.TryGetValue(nodeID, dofID, out int subdomainIdx);
					if (isRelevant)
					{
						commonDofIndices.Add(subdomainIdx);
					}
				}
				allCommonDofIndices[neighborID] = commonDofIndices.ToArray();
			}

			return LocalIndexerDto.CreateWithNewContent(subdomainDofs.NumEntries, allCommonDofIndices);
		}

		//TODOMPI: Avoid finding and storing the common nodes of a subdomain pair twice. Actually, the GetCommonNodesOfSubdomains 
		//      is safer this way.
		#region premature optimization 
		///// <summary>
		///// First key = subdomain with min id. Second key = subdomain with max id. Value = common nodes between the 2 subdomains.
		///// This way ids of the common nodes are only stored once. 
		///// </summary>
		//private readonly ConcurrentDictionary<int, ConcurrentDictionary<int, SortedSet<int>>> commonNodesWithNeighbors;

		//public SortedSet<int> GetCommonNodesOfSubdomains(int subdomain0, int subdomain1)
		//{
		//    if (subdomain0 < subdomain1)
		//    {
		//        return commonNodesWithNeighbors[subdomain0][subdomain1];
		//    }
		//    else
		//    {
		//        Debug.Assert(subdomain0 > subdomain1, "Requesting the common nodes of a subdomain with itself is illegal.");
		//        return commonNodesWithNeighbors[subdomain1][subdomain0];
		//    }
		//}
		#endregion
	}
}
