# Intro
For most solver classes the user only needs to select their parameters through a dedicated ISolverBuilder object from which the solver will be created. Afterwards the solver will be used by classes that perform the analysis.

# PCG solver
Example usage of this solver with Jacobi preconditioner.

```csharp
IModel model = ...; // created beforehand

// Set up PCG parameters
var pcgBuilder = new PcgAlgorithm.Builder();
pcgBuilder.ResidualTolerance = 1E-7; // PCG will converge once norm2(current residual)/norm2(initial residual) <= 1E-7
pcgBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(100); // If PCG iterations exceed 100, terminate the algorithm and consider it a failure

// Set up solver parameters
var solverBuilder = new PcgSolver.Builder();
solverBuilder.PcgAlgorithm = pcgBuilder.Build();
solverBuilder.PreconditionerFactory = new JacobiPreconditioner.Factory(); // Choose preconditioning strategy

// Create the solver object
PcgSolver solver = solverBuilder.BuildSolver(model);

// Setup analysis
var problem = new ProblemStructural(model, solver);
var linearAnalyzer = new LinearAnalyzer(model, solver, problem);
var staticAnalyzer = new StaticAnalyzer(model, solver, problem, linearAnalyzer);
```

To use the regular Conjugate Gradient without preconditioning, replace the corresponding line with
```csharp
solverBuilder.PreconditionerFactory = new IdentityPreconditioner.Factory();
```

Usually the acceptable number of PCG iterations depends on the size of the linear system. In that case a static number (e.g. 100 iterations in the above example) is not appropriate. We can specify that the number of PCG iterations is half the size of the linear system matrix, by replacing the corresponding line with
```csharp
pcgBuilder.MaxIterationsProvider = new PercentageMaxIterationsProvider(0.5);
```

# GMRES solver
Example usage of this solver without preconditioning.

```csharp
IModel model = ...; // created beforehand

// Set up GMRES parameters
var gmresBuilder = new GmresAlgorithm.Builder();
gmresBuilder.RelativeTolerance = 1E-7; // GMRES will converge once norm2(current residual)/norm2(initial residual) <= 1E-7
gmresBuilder.MaximumIterations = 100; // If GMRES iterations exceed 100, terminate the algorithm

// Set up solver parameters
var solverBuilder = new GmresSolver.Builder();
solverBuilder.GmresAlgorithm = gmresBuilder.Build();
solverBuilder.PreconditionerFactory = new IdentityPreconditioner.Factory(); // No preconditioning

// Create the solver object
GmresSolver solver = solverBuilder.BuildSolver(model);

// Setup analysis
var problem = new ProblemStructural(model, solver);
var linearAnalyzer = new LinearAnalyzer(model, solver, problem);
var staticAnalyzer = new StaticAnalyzer(model, solver, problem, linearAnalyzer);
```