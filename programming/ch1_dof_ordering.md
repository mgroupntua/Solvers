# Intro
Global matrix assembler classes are responsible for building the linear system matrix (global) from individual submatrices (local) that correspond to the physical model entities, such as finite elements, nodes, etc. Each assembler uses a specific [storage format](https://mgroupntua.github.io/LinearAlgebra/theory/Storage_formats.html) for the linear system matrix. The resulting matrix is then used by an appropriate solver class to solve the linear system.

# DenseMatrixAssembler
Creates the linear system matrix in full storage format. Suitable for symmetrix or nonsymmetric matrices. However, it has the largest memory requirements, thus should only be used for small problems, e.g. for debugging purposes. Examples:

