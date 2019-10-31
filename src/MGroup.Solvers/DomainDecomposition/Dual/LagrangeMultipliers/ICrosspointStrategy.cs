using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DomainDecomposition.Dual.LagrangeMultipliers
{
    /// <summary>
    /// Finds the combinations of subdomains that contain same boundary node.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public interface ICrosspointStrategy
    {
        //TODO: Perhaps a list of pairs is better than a pair of lists.
        (ISubdomain[] subdomainsPlus, ISubdomain[] subdomainsMinus) FindSubdomainCombinations(ISubdomain[] nodeSubdomains);
    }
}
