using MGroup.MSolve.Discretization.FreedomDegrees;
using MGroup.MSolve.Discretization.Interfaces;

namespace MGroup.Solvers.Ordering
{
	public interface IAsymmetricDofOrderingStrategy
	{
		(int numGlobalFreeDofs, DofTable globalFreeDofs) OrderGlobalDofs(IAsymmetricModel model);

		(int numSubdomainFreeDofs, DofTable subdomainFreeDofs) OrderSubdomainDofs(IAsymmetricSubdomain subdomain);
	}
}
