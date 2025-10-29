namespace MGroup.Solvers.DDM.FetiDP.Dofs
{
	using System.Collections.Generic;
	using System.Diagnostics;

	using MGroup.LinearAlgebra.Distributed.Overlapping;
	using MGroup.LinearAlgebra.Matrices.Operators;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.Solvers.DDM.LagrangeMultipliers;

	public class SubdomainLagranges
	{
		private readonly ICrossPointStrategy crossPointStrategy;
		private readonly IModel model;
		private readonly FetiDPSubdomainDofs subdomainDofs;
		private readonly int subdomainID;
		private readonly ISubdomainTopology subdomainTopology;

		/// <summary>
		/// Since they are sorted, they will be the same for both subdomains of each pair.
		/// </summary>
		private readonly Dictionary<int, SortedSet<LagrangeMultiplier>> commonLagrangesWithNeighbors
			= new Dictionary<int, SortedSet<LagrangeMultiplier>>();

		public SubdomainLagranges(IModel model, int subdomainID, ISubdomainTopology subdomainTopology,
			FetiDPSubdomainDofs subdomainDofs, ICrossPointStrategy crossPointStrategy)
		{
			this.model = model;
			this.subdomainID = subdomainID;
			this.subdomainTopology = subdomainTopology;
			this.subdomainDofs = subdomainDofs;
			this.crossPointStrategy = crossPointStrategy;
		}

		public List<LagrangeMultiplier> LagrangeMultipliers { get; private set; }

		//TODO: dedicated class, since in Dr, each row has exactly one +-1 entry 
		public SignedBooleanMatrixRowMajor MatrixDr { get; private set; }

		public SignedBooleanMatrixRowMajor MatrixDbr { get; private set; }

		public void CalcSignedBooleanMatrices()
		{
			IntDofTable boundaryRemainderDofs = subdomainDofs.DofOrderingBoundaryRemainder;
			int[] boundaryRemainderToRemainder = subdomainDofs.DofsBoundaryRemainderToRemainder;
			MatrixDbr = new SignedBooleanMatrixRowMajor(LagrangeMultipliers.Count, boundaryRemainderToRemainder.Length);
			MatrixDr = new SignedBooleanMatrixRowMajor(LagrangeMultipliers.Count, subdomainDofs.DofsRemainderToFree.Length);
			foreach (LagrangeMultiplier lagrange in LagrangeMultipliers)
			{
				bool positiveSign = subdomainID == lagrange.SubdomainPlus;
				int boundaryRemainderDofIdx = boundaryRemainderDofs[lagrange.NodeID, lagrange.DofID];
				MatrixDbr.AddEntry(lagrange.LocalIdx, boundaryRemainderDofIdx, positiveSign);

				int remainderDofIdx = boundaryRemainderToRemainder[boundaryRemainderDofIdx];
				MatrixDr.AddEntry(lagrange.LocalIdx, remainderDofIdx, positiveSign);
			}
		}

		public void DefineSubdomainLagrangeMultipliers()
		{
			IntDofTable boundaryRemainderDofs = subdomainDofs.DofOrderingBoundaryRemainder;
			if (LagrangeMultipliers == null)
			{
				LagrangeMultipliers = new List<LagrangeMultiplier>(boundaryRemainderDofs.NumEntries);
			}
			else
			{
				LagrangeMultipliers.Clear();

			}

			foreach (int nodeID in boundaryRemainderDofs.GetRows())
			{
				INode node = model.GetNode(nodeID);
				IList<(int subdomainPlus, int subdomainMinus)> combos = ListSubdomainCombinations(node);
				foreach (int dofID in boundaryRemainderDofs.GetColumnsOfRow(nodeID))
				{
					foreach ((int sPlus, int sMinus) in combos)
					{
						int localIdx = LagrangeMultipliers.Count;
						LagrangeMultipliers.Add(new LagrangeMultiplier(nodeID, dofID, sPlus, sMinus, localIdx));
					}
				}
			}
		}

		public void FindCommonLagrangesWithNeighbors()
		{
			commonLagrangesWithNeighbors.Clear();
			foreach (LagrangeMultiplier lagrange in LagrangeMultipliers)
			{
				Debug.Assert((this.subdomainID == lagrange.SubdomainPlus) || (this.subdomainID == lagrange.SubdomainMinus));
				int neighborID = (lagrange.SubdomainPlus == this.subdomainID) ? lagrange.SubdomainMinus : lagrange.SubdomainPlus;
				if (!commonLagrangesWithNeighbors.TryGetValue(neighborID, out SortedSet<LagrangeMultiplier> commonLagranges))
				{
					commonLagranges = new SortedSet<LagrangeMultiplier>();
					commonLagrangesWithNeighbors[neighborID] = commonLagranges;
				}
				commonLagranges.Add(lagrange);
			}

			// The next does not necessarily hold (e.g. in 2D), since all common dofs between 2 subdomains may be corner dofs, 
			// instead of having lagrange multipliers applied.
			//Debug.Assert(subdomainTopology.GetNeighborsOfSubdomain(subdomainID).SequenceEqual(commonLagrangesWithNeighbors.Keys));
		}

		public LocalIndexerDto InitializeDistributedVectorIndexer()
		{
			var allCommonLagrangeIndices = new Dictionary<int, int[]>();
			foreach (int neighborID in commonLagrangesWithNeighbors.Keys)
			{
				SortedSet<LagrangeMultiplier> commonLagranges = commonLagrangesWithNeighbors[neighborID];
				var commonLagrangeIndices = new int[commonLagranges.Count];
				int i = 0;
				foreach (LagrangeMultiplier lagrange in commonLagranges)
				{
					commonLagrangeIndices[i++] = lagrange.LocalIdx;
				}

				allCommonLagrangeIndices[neighborID] = commonLagrangeIndices;
			}

			return LocalIndexerDto.CreateWithNewContent(LagrangeMultipliers.Count, allCommonLagrangeIndices);
		}

		private IList<(int subdomainPlus, int subdomainMinus)> ListSubdomainCombinations(INode node)
		{
			if (node.Subdomains.Count == 2)
			{
				var combos = new (int subdomainPlus, int subdomainMinus)[1];
				int s0 = node.Subdomains.First();
				int s1 = node.Subdomains.Last();
				if (s0 < s1)
				{
					combos[0] = (s0, s1);
				}
				else
				{
					combos[0] = (s1, s0);
				}
				return combos;
			}
			else
			{
				Debug.Assert(node.Subdomains.Count > 2, "Found boundary remainder node with less than 2 subdomains");
				List<(int subdomainPlus, int subdomainMinus)> combos = crossPointStrategy.ListSubdomainCombinations(node.Subdomains);
				combos.RemoveAll(c => (c.subdomainPlus != this.subdomainID) && (c.subdomainMinus != this.subdomainID));
				return combos;
			}
		}
	}
}
