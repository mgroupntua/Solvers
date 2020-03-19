# Intro
Iterative solvers for a linear system `A * x = b` start from an intial guess and refine it in a series of iterations, until the solution of the system is approximated with sufficient accuracy. In each iteration only matrix-vector multiplications and vector-vector operations are performed, as well as preconditioning operations. Therefore the linear system matrix does not need to be factorized. In fact the system matrix does not have to be formed explicitly. Instead only the matrix-vector multiplication is needed, which makes iterative algorithms necessary for matrix-free and domain decomposition methods.

# Performance
The efficiency of iterative solvers does not depend on the bandwidth of the system matrix, as much as direct solvers. Therefore they are a good candidate for problems that result in matrices with large bandwidth, such as 3D FEM problems. Nevertheless, their convergence rate is very sensitive to the condition of the matrix, thus preconditioning techniques are used to reduce the number of iterations. Unfortunately finding a preconditioner that can both be implemented efficiently and reduce the iterations significantly is a challenging task. There is a number of general purpose preconditioners, but the most competitive ones are usually problem dependent.

# PCG solver
Uses the Preconditioned Conjugate Gradient algorithm to solve the linear system. This solver is applicable only for symmetric positive definite matrices. For more details see [Painless Conjugate Gradient](https://www.cs.cmu.edu/~quake-papers/painless-conjugate-gradient.pdf).

# MINRES solver
Uses the Minimum Residual method with preconditioning to solve the linear system. This solver is applicable only for symmetric matrices. For symmetric positive definite matrices PCG is usually more efficient. For more details see [Minimum Residual Method](http://mathworld.wolfram.com/MinimalResidualMethod.html). 

# GMRES solver
Uses the Generalized Minimum Residual method with preconditioning to solve the linear system. This solver is applicable for both symmetric and nonsymmetric matrices. For symmetric positive definite matrices PCG and MINRES are usually more efficient. For more details, see [Iterative Methods for Sparse Linear Systems](https://www-users.cs.umn.edu/~saad/IterMethBook_2ndEd.pdf).

# Preconditioning
Preconditioners are matrices (or equivalent matrix-vector operations) that are almost always used in an iterative solver to reduce the number of iterations required for convergence. Preconditioners must strike a good balance between two coonflicting objectives:
1. Computationally efficient: Fast to compute, fast to implement at each iteration, and with low memory requirements to store.
2. Effective at reducing the number of iterations, which is problem dependent.

The available preconditioners are:

## Jacobi or diagonal preconditioner
This preconditioner is obtained by inverting the diagonal of the linear system matrix. It is very computationally efficient, but does not improve the convergence rate significantly, unless the diagonal entries of the linear system matrix are much larger than the non-diagonal ones. For more details see [Iterative Methods for Sparse Linear Systems](https://www-users.cs.umn.edu/~saad/IterMethBook_2ndEd.pdf).

## Preconditioners for Domain Decomposition Methods
These are presented along with the corresponding DDM in later chapters. 