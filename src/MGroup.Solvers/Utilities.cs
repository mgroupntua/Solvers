namespace MGroup.Solvers
{
	public static class Utilities
	{
		public static bool AreEqual(int[] array1, int[] array2)
		{
			if (array1.Length != array2.Length) return false;
			for (int i = 0; i < array1.Length; ++i)
			{
				if (array1[i] != array2[i]) return false;
			}
			return true;
		}

		public static int[] Range(int startInclusive, int endExclusive)
		{
			var result = new int[endExclusive - startInclusive];
			for (int i = startInclusive; i < endExclusive; ++i)
			{
				result[i] = i;
			}

			return result;
		}
	}
}
