using Xunit;

namespace MGroup.Solvers.Tests
{
	internal static class Utilities
    {
        internal static void CheckEqual(int[] expected, int[] computed)
        {
            Assert.Equal(expected.Length, computed.Length);
            for (int i = 0; i < expected.Length; ++i) Assert.Equal(expected[i], computed[i]);
        }
    }
}
