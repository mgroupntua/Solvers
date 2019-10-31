using MGroup.MSolve.Discretization;

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
