using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.DomainDecomposition.MeshPartitioning
{
    public enum NodePosition
    {
        Internal, Boundary, External
    }

    public interface IRegion2D
    {
        NodePosition FindRelativePosition(INode node);
    }
}
