# Intro
Direct solvers for a linear system `A * x = b` comprise 2-3 steps:
1. (Optional) Find and apply a permutation of the rows/columns that will lead to fewer operations in the next two steps.
2. The linear system matrix is factorized, i.e. decomposed as the product of two simpler (usually triangular) matrices. 
    - E.g., if LU factorization is used: `A = L * U`.
3. The factorization matrices are then used to solve the original linear system.
    - E.g., in the previous example: 
    ```
    A * x = b <=> L * (U*x) = b <=> L * y = b
    Solve y = inv(L) * b (perform forward substitution, instead of inversing L)
    Solve x = inv(U) * y (perform back substitution, instead of inversing U)
    ```

# Performance
The factorization step is the most computationally intensive, while the solution step is relatively fast. Therefore, direct solvers are recommended when nearby problems are solved, namely linear systems with the same matrix and various right hand side vectors. In this case, the matrix is factorized only once and only the solution step is performed for each rhs vector.

The performance of direct solvers greatly depends on the sparsity of the linear system matrix. Sparse matrix formats appropriate for each factorization algorithm are used for the matrix. The sparsity pattern determines the bandwidth of the matrix, that is the number of non-zero entries of each row/colum required to represent the factorization matrices. If the bandwidth is large, direct solvers are inefficient or even inapplicable due to excessive memory requirements. The bandwidth of the matrix resutling from FEM discretization is largest for 3D problems, then 2D problems, then 1D problems. 

# Dense LUP solver
This solver creates the linear system matrix `A` in full matrix format. The factorization step uses the LUP factorization algorithm `A = P*L*U`, where `P (n x n)` is a permutation matrix. The matrices `L,U` can overwrite the original matrix `A`. The solution step performs back&forward substitutions with the matrices `L,U,P`. 

This solver is applicable for all invertible matrices. It does not take advantage of the sparsity pattern of a matrix. Therefore it can only be used for small linear systems.

For more details see [Full format](https://mgroupntua.github.io/LinearAlgebra/theory/Storage_formats.html) and [LUP factorization](https://mgroupntua.github.io/LinearAlgebra/theory/Direct_system_solution.html). 


# Skyline LDL solver
This solver creates the linear system matrix `A` in Skyline matrix format. The factorization step uses the LDL factorization algorithm `A=L*D*L^T` and the matrix `L` can overwrite the original matrix `A`. The solution step performs back&forward substitutions with the matrix `L`. 

This solver is only applicable for symmetric positive definite matrices. The Skyline matrix format is very efficient for LDL factorization, since it stores exactly the number of entries needed for the matrix `L`. However, if the bandwidth is very large, the creation of the matrix `A` may not be computed, due to insufficient memory.

For more details see [Skyline format](https://mgroupntua.github.io/LinearAlgebra/theory/Storage_formats.html) and [LDL factorization](https://mgroupntua.github.io/LinearAlgebra/theory/Direct_system_solution.html). 

# Reordering
The bandwidth of sparse direct solvers (e.g. Skyline LDL) can be improved by applying a reordering algorithm that calculates a fill reducing permutation for the rows/columns of the linear system matrix. This reordering can be applied prior to forming the matrix, since it only needs the sparsity pattern, which only depends on the connectivity of the finite elements. Therefore finding the sparsity pattern is a very fast operation, storing it requires little memory and applying the reordering algorithm is much faster than the factorization and solution steps. After identifying a fill reducing permutation, it is applied to the numbering of freedom degrees of the whole problem, namely the nodes and elements of the FE model, instead of only when the sparse direct solver is used. 

The available reordering algorithms are:

## Approximate Minimum Degree (AMD)
The Approximate Minimum Degree is a heuristic method that finds a fill-reducing permutation of the matrix **C = A + A<sup>T</sup>** where **A** is the linear system matrix. It is computationally efficent and usually results in low fill-in for a variety of matrices. For more details see [An Approximate Minimum Degree Ordering Algorithm](http://faculty.cse.tamu.edu/davis/publications_files/An_Approximate_Minimum_Degree_Ordering_Algorithm.pdf).

## Constrained Approximate Minimum Degree (CAMD)
Constrained Approximate Minimum Degree is a variant of AMD, that allows specifying groups of rows/columns. During the calculation of a fill-reducing permutation, CAMD will place rows/columns of the same group consecutively, without mixing rows/columns from different groups. This results in a worse permutation overall, but may be required by some applications.
