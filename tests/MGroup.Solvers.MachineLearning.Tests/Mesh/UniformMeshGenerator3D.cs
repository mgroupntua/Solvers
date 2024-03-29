//TODO: abstract this in order to be used with points in various coordinate systems
//TODO: perhaps the origin should be (0.0, 0.0) and the meshes could then be transformed. Abaqus does something similar with its
//      meshed parts during assembly
namespace MGroup.Solvers.MachineLearning.Tests.Mesh
{
	using System;
	using System.Collections.Generic;
	using System.Linq;
	using System.Text;
	using System.Threading.Tasks;

	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.Entities;
	using MGroup.MSolve.Discretization.Meshes.Generation;

	/// <summary>
	/// Creates 3D meshes based on uniform rectilinear grids: the distance between two consecutive vertices for the same axis is 
	/// constant. This distance may be different for each axis though. For now the cells are hexahedral with 8 vertices. 
	/// (bricks in particular).
	/// </summary>
	public class UniformMeshGenerator3D<TNode> : IMeshGenerator<TNode> where TNode : INode
	{
		private readonly double minX, minY, minZ;
		private readonly double dx, dy, dz;
		private readonly int cellsPerX, cellsPerY, cellsPerZ;
		private readonly int verticesPerX, verticesPerY, verticesPerZ;

		public UniformMeshGenerator3D(double minX, double minY, double minZ, double maxX, double maxY, double maxZ,
			int cellsPerX, int cellsPerY, int cellsPerZ)
		{
			this.minX = minX;
			this.minY = minY;
			this.minZ = minZ;
			this.dx = (maxX - minX) / cellsPerX;
			this.dy = (maxY - minY) / cellsPerY;
			this.dz = (maxZ - minZ) / cellsPerZ;
			this.cellsPerX = cellsPerX;
			this.cellsPerY = cellsPerY;
			this.cellsPerZ = cellsPerZ;
			this.verticesPerX = cellsPerX + 1;
			this.verticesPerY = cellsPerY + 1;
			this.verticesPerZ = cellsPerZ + 1;
		}

		public bool StartIDsAt0 { get; set; } = true;

		/// <summary>
		/// Generates a uniform mesh with the dimensions and density defined in the constructor.
		/// </summary>
		/// <returns></returns>
		public (IReadOnlyList<TNode> nodes, IReadOnlyList<CellConnectivity<TNode>> elements)
			CreateMesh(CreateNode<TNode> createNode)
		{
			TNode[] nodes = CreateNodes(createNode);
			CellConnectivity<TNode>[] elements = CreateElements(nodes);
			return (nodes, elements);
		}

		private TNode[] CreateNodes(CreateNode<TNode> createNode)
		{
			var vertices = new TNode[verticesPerX * verticesPerY * verticesPerZ];
			int id = 0;
			int start = StartIDsAt0 ? 0 : 1;
			for (int k = 0; k < verticesPerZ; ++k)
			{
				for (int j = 0; j < verticesPerY; ++j)
				{
					for (int i = 0; i < verticesPerX; ++i)
					{
						vertices[id] = createNode(start + id, minX + i * dx, minY + j * dy, minZ + k * dz);
						++id;
					}
				}
			}
			return vertices;
		}

		private CellConnectivity<TNode>[] CreateElements(TNode[] allVertices)
		{
			var cells = new CellConnectivity<TNode>[cellsPerX * cellsPerY * cellsPerZ];
			for (int k = 0; k < cellsPerZ; ++k)
			{
				for (int j = 0; j < cellsPerY; ++j)
				{
					for (int i = 0; i < cellsPerX; ++i)
					{
						int cell = k * cellsPerX * cellsPerY + j * cellsPerX + i;
						int firstVertex = k * verticesPerX * verticesPerY + j * verticesPerX + i;
						//int[] verticesOfCell = Hexa8VerticesConventional(verticesPerX, verticesPerY, firstVertex);
						int[] verticesOfCell = Hexa8VerticesBathe(verticesPerX, verticesPerY, firstVertex);
						cells[cell] = new CellConnectivity<TNode>(CellType.Hexa8,
							verticesOfCell.Select(idx => allVertices[idx]).ToArray()); // row major
					}
				}
			}
			return cells;
		}

		private static int[] Hexa8VerticesBathe(int verticesPerX, int verticesPerY, int firstVertex)
		{
			var vertices = new int[]											// Node order for Hexa8
			{
				firstVertex + verticesPerX * verticesPerY + verticesPerX + 1,   // ( 1,  1,  1)
				firstVertex + verticesPerX * verticesPerY + verticesPerX,       // (-1,  1,  1)
				firstVertex + verticesPerX * verticesPerY,                      // (-1, -1,  1)
				firstVertex + verticesPerX * verticesPerY + 1,                  // ( 1, -1,  1)
				firstVertex + verticesPerX + 1,                                 // ( 1,  1, -1)
				firstVertex + verticesPerX,                                     // (-1,  1, -1)
				firstVertex,                                                    // (-1, -1, -1)
				firstVertex + 1,                                                // ( 1, -1, -1)
			};
			return vertices;
		}

		private static int[] Hexa8VerticesConventional(int verticesPerX, int verticesPerY, int firstVertex)
		{
			var vertices = new int[]											// Node order for Hexa8
			{
				firstVertex,                                                    // (-1, -1, -1)
				firstVertex + 1,                                                // ( 1, -1, -1)
				firstVertex + verticesPerX + 1,                                 // ( 1,  1, -1)
				firstVertex + verticesPerX,                                     // (-1,  1, -1)
				firstVertex + verticesPerX * verticesPerY,                      // (-1, -1,  1)
				firstVertex + verticesPerX * verticesPerY + 1,                  // ( 1, -1,  1)
				firstVertex + verticesPerX * verticesPerY + verticesPerX + 1,   // ( 1,  1,  1)
				firstVertex + verticesPerX * verticesPerY + verticesPerX        // (-1,  1,  1)
			};
			return vertices;
		}
	}
}
