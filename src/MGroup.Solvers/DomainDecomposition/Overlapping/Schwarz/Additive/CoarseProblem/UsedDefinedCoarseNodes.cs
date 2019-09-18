using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MGroup.Solvers.DomainDecomposition.Overlapping.Schwarz.Additive.CoarseProblem
{
    public class UsedDefinedCoarseNodes:ICoarseSpaceNodeSelection
    {
        private readonly int _numberOfControlPoints;
        private readonly int[] _constrainedDofs;
        private readonly int[] _freeDofs;

        public UsedDefinedCoarseNodes(int numberOfControlPoints, int[] constrainedDofs)
        {
            _numberOfControlPoints = numberOfControlPoints;
            _constrainedDofs = constrainedDofs.ToArray();
            var allDofs=Enumerable.Range(0,_numberOfControlPoints);
            _freeDofs = allDofs.Where(dof => !constrainedDofs.Contains(dof)).ToArray();
        }


        public int[] GetConstrainedCoarseSpaceDofs() => _constrainedDofs;

        public int[] GetFreeCoarseSpaceDofs() => _freeDofs;
    }
}
