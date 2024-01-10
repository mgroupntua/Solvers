using MGroup.Solvers.MachineLearning.Tests.Examples;

using Xunit;

namespace MGroup.Solver.MachineLearning.Tests
{
    public class StructuralTests
    {
		[Fact(Skip = "Takes too long to complete in Azure DevOps")]
        public static void Elasticity3DTest() => Elasticity3DExample.Run();
    }
}
