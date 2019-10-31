using MGroup.LinearAlgebra.Vectors;

//TODO: There are various convergence criteria. See Fragakis PhD, section 4.3.1 page 93/299. 
namespace MGroup.Solvers.DomainDecomposition.Dual.Pcg
{
    /// <summary>
    /// Called by PCPG.
    /// </summary>
    internal delegate double CalculateExactResidualNorm(Vector lagrangeMultipliers);

    internal interface IExactResidualCalculator
    {
        /// <summary>
        /// Called by FetiSolver after calculating the global displacements.
        /// </summary>
        /// <param name="globalDisplacements"></param>
        double CalculateExactResidualNorm(IVectorView globalDisplacements);
    }
}
