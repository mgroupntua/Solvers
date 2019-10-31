using System.Collections.Generic;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DomainDecomposition.Dual.FetiDP.CornerNodes
{
    public interface ICornerNodeSelection
    {
        //TODO: These should probably be HashSet instead of array
        Dictionary<int, HashSet<INode>> SelectCornerNodesOfSubdomains();
    }
}
