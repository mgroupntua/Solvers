### About
Here you can find all information pertaining to the theory applied for the implementation of this library.

### Table of contents
- [Chapter 1 - Direct solvers](theory/ch1_direct_solvers.md)
- [Chapter 2 - Iterative solvers](theory/ch2_iterative_solvers.md)
- [Chapter 3 - Solvers based on Domain Decomposition Methods](theory/ch3_ddm_solvers.md)

### Solver selection
Some general guidelines to choose the correct solver for a problem are:
- For 1D problems, direct solvers are usually recommended.
- For 2D problems, direct solvers are often the most efficient. However other options could be preferable.
- For 3D problems, iterative solvers with appropriate preconditioners or domain decomposition methods are recommended. 