# Intro
Global matrix assembler classes are responsible for building the linear system matrix (global or subdomain level) from individual submatrices (local level) that correspond to the physical model entities, such as finite elements, nodes, etc. Each assembler uses a specific [storage format](https://mgroupntua.github.io/LinearAlgebra/theory/Storage_formats.html) for the linear system matrix. The resulting matrix is then used by an appropriate solver class to solve the linear system. The solver class usually specifies which matrix assembler it needs, so that the user does not have to.

# DenseMatrixAssembler
Creates the linear system matrix in Full storage format. Suitable for symmetric or nonsymmetric matrices, in conjuction with dense direct solvers. However, it has the largest memory requirements, thus should only be used for small problems, e.g. for debugging purposes. Example:

```csharp
ISubdomainFreeDofOrdering dofOrdering = subdomain.FreeDofOrdering; // The order of the freedom degrees of the problem. Created by dof ordering classes.
IEnumerable<IElement> elements = subdomain.Elements; // Accessed from subdomain
IElementMatrixProvider elementMatrixProvider = new ElemestStiffnessMatrixProvider(); // For elasticity problems

DenseMatrixAssembler assembler = new DenseMatrixAssembler();
Matrix globalMatrix = assembler.BuildGlobalMatrix(dofOrdering, elements, elementMatrixProvider);
```

# SkylineAssembler
Creates the linear system matrix in Skyline format. Suitable for symmetric matrices, in conjuction with sparse direct solvers. The entries stored are exactly those needed for factorizing the matrix, e.g. by a direct sparse solver, thus its memory requirements greatly depend on the bandwidth. Example:

```csharp
ISubdomainFreeDofOrdering dofOrdering = subdomain.FreeDofOrdering; // The order of the freedom degrees of the problem. Created by dof ordering classes.
IEnumerable<IElement> elements = subdomain.Elements; // Accessed from subdomain
IElementMatrixProvider elementMatrixProvider = new ElemestStiffnessMatrixProvider(); // For elasticity problems

SkylineAssembler assembler = new SkylineAssembler();
SkylineMatrix globalMatrix = assembler.BuildGlobalMatrix(dofOrdering, elements, elementMatrixProvider);
```

# SymmetricCscAssembler 
Creates the linear system matrix in CSC format, but only the entries of the upper triangle are stored. Suitable for symmetric matrices, in conjuction with sparse direct solvers. Its memory requirements are very low, since it only stores nonzero entries. However it stores less entries than required be a direct sparse solver, thus the actual memory requirements will be larger during the solution phase. Example:

```csharp
ISubdomainFreeDofOrdering dofOrdering = subdomain.FreeDofOrdering; // The order of the freedom degrees of the problem. Created by dof ordering classes.
IEnumerable<IElement> elements = subdomain.Elements; // Accessed from subdomain
IElementMatrixProvider elementMatrixProvider = new ElemestStiffnessMatrixProvider(); // For elasticity problems

bool sortRowsOfEachCol = true; // The CSC matrix will have its columns sorted, which may improve the performance of the solver, for a small overhead during the matrix assembly.
SymmetricCscAssembler assembler = new SymmetricCscAssembler(sortRowsOfEachCol);
SymmetricCscMatrix  globalMatrix = assembler.BuildGlobalMatrix(dofOrdering, elements, elementMatrixProvider);
```

# CsrAssembler  
Creates the linear system matrix in CSR format. Suitable for symmetric and nonsymmetric matrices, in conjuction with iterative solvers. Its memory requirements are very low, since it only stores nonzero entries. Example:

```csharp
ISubdomainFreeDofOrdering dofOrdering = subdomain.FreeDofOrdering; // The order of the freedom degrees of the problem. Created by dof ordering classes.
IEnumerable<IElement> elements = subdomain.Elements; // Accessed from subdomain
IElementMatrixProvider elementMatrixProvider = new ElemestStiffnessMatrixProvider(); // For elasticity problems

bool sortColsOfEachRow = true; // The CSR matrix will have its rows sorted, which will improve the performance of the solver, for a small overhead during the matrix assembly.
CsrAssembler assembler = new CsrAssembler(sortColsOfEachRow);
CsrMatrix globalMatrix = assembler.BuildGlobalMatrix(dofOrdering, elements, elementMatrixProvider);
```

# CsrNonSymmetricAssembler  
Creates the linear system matrix in CSR format. Contrary to CsrAssembler, the rows of the matrix do not correspond to the same freedom degrees as the columns. Suitable for nonsymmetric matrices, in conjuction with iterative solvers. Its memory requirements are very low, since it only stores nonzero entries. Example:

```csharp
ISubdomainFreeDofOrdering rowDofOrdering = subdomain.FreeDofOrdering; // The order of the freedom degrees of the problem that correspond to rows of the linear system matrix. Created by dof ordering classes.
ISubdomainFreeDofOrdering colDofOrdering = subdomain.FreeDofOrdering; // The order of the freedom degrees of the problem that correspond to columns of the linear system matrix. Created by dof ordering classes.
IEnumerable<IElement> elements = subdomain.Elements; // Accessed from subdomain
IElementMatrixProvider elementMatrixProvider = new ElemestStiffnessMatrixProvider(); // For elasticity problems

bool sortColsOfEachRow = true; // The CSR matrix will have its rows sorted, which will improve the performance of the solver, for a small overhead during the matrix assembly.
CsrNonSymmetricAssembler assembler = new CsrNonSymmetricAssembler(sortColsOfEachRow);
CsrMatrix globalMatrix = assembler.BuildGlobalMatrix(rowDofOrdering, colDofOrdering,  elements, elementMatrixProvider);
```