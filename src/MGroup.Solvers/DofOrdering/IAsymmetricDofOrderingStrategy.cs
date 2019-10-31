using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DofOrdering
{
	public interface IAsymmetricDofOrderingStrategy
	{
		(int numGlobalFreeDofs, DofTable globalFreeDofs) OrderGlobalDofs(IAsymmetricModel model);
		(int numSubdomainFreeDofs, DofTable subdomainFreeDofs) OrderSubdomainDofs(IAsymmetricSubdomain subdomain);
	}
}
