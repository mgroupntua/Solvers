namespace MGroup.Solvers.DofOrdering.Reordering
{
    public static class ReorderingUtilities
    {
        public static int[] ReorderKeysOfDofIndicesMap(int[] dofIndicesMap, int[] permutation, bool oldToNew)
        {
            int numDofs = permutation.Length;
            var result = new int[numDofs];
            if (oldToNew)
            {
                for (int i = 0; i < numDofs; ++i) result[permutation[i]] = dofIndicesMap[i]; // i is old index
            }
            else
            {
                for (int i = 0; i < numDofs; ++i) result[i] = dofIndicesMap[permutation[i]]; // i is new index
            }
            return result;
        }
    }
}
