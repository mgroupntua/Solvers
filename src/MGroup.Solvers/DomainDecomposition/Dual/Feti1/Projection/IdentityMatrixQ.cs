using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DomainDecomposition.Dual.Feti1.Projection
{
    internal class IdentityMatrixQ : IMatrixQ
    {
        public Vector Multiply(Vector vector) => vector; //TODO: Should I copy it?
        public Matrix Multiply(Matrix matrix) => matrix; //TODO: Should I copy it? G^T*G might be optimizable if G is the same object.
    }
}
