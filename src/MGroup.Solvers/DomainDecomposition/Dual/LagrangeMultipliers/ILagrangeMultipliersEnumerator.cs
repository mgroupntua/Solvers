using System.Collections.Generic;
using MGroup.LinearAlgebra.Matrices.Operators;

namespace MGroup.Solvers.DomainDecomposition.Dual.LagrangeMultipliers
{
    public interface ILagrangeMultipliersEnumerator
    {
        Dictionary<int, SignedBooleanMatrixColMajor> BooleanMatrices { get; }

        //TODO: I am not too thrilled about objects with properties that may or may not be null
        /// <summary>
        /// WARNING: This property will be null in homogeneous problems.
        /// Associates each lagrange multiplier with the instances of the boundary dof, for which continuity is enforced. 
        /// </summary>
        LagrangeMultiplier[] LagrangeMultipliers { get; }

        int NumLagrangeMultipliers { get; }
    }
}
