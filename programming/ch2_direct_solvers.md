# Intro
For most solver classes the user only needs to select their parameters through a dedicated ISolverBuilder object from which the solver will be created. Afterwards the solver will be used by classes that perform the analysis.

# Dense solver
Example usage of this solver

```csharp
IModel model = ...; // created beforehand

// Set up the solver's parameters
var solverBuilder = new DenseMatrixSolver.Builder();
solverBuilder.IsMatrixPositiveDefinite = true; // true to use LDL factorization, false to use LUP factorization. LUP is applicable to all invertible matrices, but slower.

// Create the solver object
DenseMatrixSolver solver = solverBuilder.BuildSolver(model);

// Setup analysis
var problem = new ProblemStructural(model, solver);
var linearAnalyzer = new LinearAnalyzer(model, solver, problem);
var staticAnalyzer = new StaticAnalyzer(model, solver, problem, linearAnalyzer);
```

# Skyline LDL solver
Example usage of this solver with AMD reordering
```csharp
IModel model = ...; // created beforehand

// Set up the solver's parameters
var solverBuilder = new DenseMatrixSolver.Builder();
solverBuilder.DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), AmdReordering.CreateWithCSparseAmd()); // AMD reordering
solverBuilder.FactorizationPivotTolerance = 1E-7; // if a diagonal (pivot) entry falls beneath this value during factorization, the matrix will not be considered positive definite and the algorithm will terminate

// Create the solver object
SkylineSolver solver = solverBuilder.BuildSolver(model);

// Setup analysis
var problem = new ProblemStructural(model, solver);
var linearAnalyzer = new LinearAnalyzer(model, solver, problem);
var staticAnalyzer = new StaticAnalyzer(model, solver, problem, linearAnalyzer);
```

If the user does not want to use any reordering algorithm, then the corresponding line should be replaced with 
```csharp
solverBuilder.DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NullReordering());
```
