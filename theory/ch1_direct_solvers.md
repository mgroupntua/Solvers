# Intro
Direct solvers for a linear system `A * x = b` comprise 2 steps:
1. The linear system matrix is factorized, i.e. decomposed as the product of two simpler (usually triangular) matrices. 
    - E.g., if LU factorization is used: `A = L * U`.
2. The factorization matrices are then used to solve the original linear system.
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

This solver does not take advantage of the sparsity pattern of a matrix. Therefore it can only be used for small linear systems.

For more details see [Full format](https://mgroupntua.github.io/LinearAlgebra/theory/Storage_formats.html) and [LUP factorization](https://mgroupntua.github.io/LinearAlgebra/theory/Direct_system_solution.html). 


# Skyline LDL solver
This solver creates the linear system matrix `A` in Skyline matrix format. The factorization step uses the LDL factorization algorithm `A=L*D*L^T` and the matrix `L` can overwrite the original matrix `A`. The solution step performs back&forward substitutions with the matrix `L`. 

The Skyline matrix format is very efficient for LDL factorization, since it stores exactly the number of entries needed for the matrix `L`. However, if the bandwidth is very large, the creation of the matrix `A` may not be computed, due to insufficient memory.

For more details see [Skyline format](https://mgroupntua.github.io/LinearAlgebra/theory/Storage_formats.html) and [LDL factorization](https://mgroupntua.github.io/LinearAlgebra/theory/Direct_system_solution.html). 
