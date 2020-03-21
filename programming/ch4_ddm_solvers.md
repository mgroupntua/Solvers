# Intro
For most solver classes the user only needs to select their parameters through a dedicated ISolverBuilder object from which the solver will be created. Afterwards the solver will be used by classes that perform the analysis.

# FETI-1 solver
Example usage of this solver for a 2nd order PDE in a serial computing environment:

```csharp
IModel model = ...; // must have more than 1 subdomains

// LDL factorization of each subdomain's Kii (stiffness matrix for internal dofs) needs a problem dependent tolerance to recognize rigid body motions
var factorizationTolerances = new Dictionary<int, double>();
foreach (Subdomain s in multiSubdomainModel.Subdomains) factorizationTolerances[s.ID] = 1E-4;

// Subdomain matrices will be stored in Full format
var fetiMatrices = new DenseFeti1SubdomainMatrixManager.Factory(); 

// Create solver builder object to set up FETI-1 parameters
var solverBuilder = new Feti1Solver.Builder(fetiMatrices, factorizationTolerances);

// The stiffness distribution among subdomains is homogeneous.
solverBuilder.ProblemIsHomogeneous = true;

// Dirichlet preconditioning
solverBuilder.PreconditionerFactory = new DirichletPreconditioner.Factory();

// PCG parameters for the interface problem solution
var interfaceProblemSolverBuilder = new Feti1ProjectedInterfaceProblemSolver();
interfaceProblemSolverBuilder.PcgConvergenceTolerance = 1E-7;
interfaceProblemSolverBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(50); // If PCG iterations exceed 50, terminate the whole DDM algorithm and consider it a failure
solverBuilder.InterfaceProblemSolver = interfaceProblemSolverBuilder.Build();

// Create the solver object
Feti1Solver solver = solverBuilder.BuildSolver(model);

// Setup analysis
var problem = new ProblemStructural(model, solver);
var linearAnalyzer = new LinearAnalyzer(model, solver, problem);
var staticAnalyzer = new StaticAnalyzer(model, solver, problem, linearAnalyzer);
```

To use Skyline format with or without AMD reordering for subdomain level matrices, in order to siginificantly speed up the computations, replace the corresponding line with
```csharp
var fetiMatrices = new SkylineFeti1SubdomainMatrixManager.Factory(new OrderingAmdCSparseNet());
```
```csharp
var fetiMatrices = new SkylineFeti1SubdomainMatrixManager.Factory();
```

To use lumped or diagonal Dirichlet preconditioning, replace the corresponding line with one of the following
```csharp
solverBuilder.PreconditionerFactory = new LumpedPreconditioner.Factory();
```
```csharp
solverBuilder.PreconditionerFactory = new DiagonalDirichletPreconditioner.Factory();
```

Usually the acceptable number of PCG iterations during the interface problem solution depends on the size of the linear system. In that case a static number (e.g. 50 iterations in the above example) is not appropriate. We can specify that the number of PCG iterations is half the size of the interface problem matrix, by replacing the corresponding line with
```csharp
interfaceProblemSolverBuilder.MaxIterationsProvider = new PercentageMaxIterationsProvider(0.5);
```

For problems where the stiffness distribution is not the same among subdomains, the following changes are necessary:
```csharp
solverBuilder.ProblemIsHomogeneous = false;
solverBuilder.ProjectionMatrixQIsIdentity = false;
```

For 4th order PDEs, the following addition is necessary:
```csharp
solverBuilder.ProjectionMatrixQIsIdentity = false;
```

# FETI-DP solver
Example usage of this solver for a 2-dimensional 2nd order PDE or a 4th order PDE in a serial computing environment:

```csharp
IModel model = ...; // must have more than 1 subdomains

// The corner nodes of each subdomain must be specified. Here subdomains are rectangles and all their corners
// will be considered. In practice some of these are constrained and should not be included.
var cornerNodesOfEachSubdomain = new Dictionary<int, HashSet<INode>>();
foreach (Subdomain subdomain in multiSubdomainModel.Subdomains)
{
    subdomain.DefineNodesFromElements(); //TODO: This will also be called by the analyzer.
    INode[] corners = CornerNodeUtilities.FindCornersOfRectangle2D(subdomain);
    cornerNodesOfEachSubdomain[subdomain.ID] = new HashSet<INode>(corners);
}
var cornerNodeSelection = new UsedDefinedCornerNodes(cornerNodesOfEachSubdomain);

// Subdomain matrices will be stored in Full format
var fetiMatrices = new DenseFetiDPSubdomainMatrixManager.Factory();

// Create solver builder object to set up FETI-DP parameters
var solverBuilder = new FetiDPSolver.Builder(cornerNodeSelection, fetiMatrices);

// The stiffness distribution among subdomains is homogeneous.
solverBuilder.ProblemIsHomogeneous = true;

// Dirichlet preconditioning
solverBuilder.PreconditionerFactory = new DirichletPreconditioner.Factory();

// PCG parameters for the interface problem solution
var interfaceProblemSolverBuilder = new FetiDPInterfaceProblemSolver();
interfaceProblemSolverBuilder.PcgConvergenceTolerance = 1E-7;
interfaceProblemSolverBuilder.MaxIterationsProvider = new FixedMaxIterationsProvider(50); // If PCG iterations exceed 50, terminate the whole DDM algorithm and consider it a failure
solverBuilder.InterfaceProblemSolver = interfaceProblemSolverBuilder.Build();

// Create the solver object
FetiDPSolver solver = solverBuilder.BuildSolver(model);

// Setup analysis
var problem = new ProblemStructural(model, solver);
var linearAnalyzer = new LinearAnalyzer(model, solver, problem);
var staticAnalyzer = new StaticAnalyzer(model, solver, problem, linearAnalyzer);
```

To use Skyline format with or without AMD reordering for subdomain level matrices, in order to siginificantly speed up the computations, replace the corresponding line with
```csharp
var fetiMatrices = new SkylineFetiDPSubdomainMatrixManager.Factory(new OrderingAmdCSparseNet());
```
```csharp
var fetiMatrices = new SkylineFetiDPSubdomainMatrixManager.Factory();
```

To use lumped or diagonal Dirichlet preconditioning, replace the corresponding line with one of the following
```csharp
solverBuilder.PreconditionerFactory = new LumpedPreconditioner.Factory();
```
```csharp
solverBuilder.PreconditionerFactory = new DiagonalDirichletPreconditioner.Factory();
```

Usually the acceptable number of PCG iterations during the interface problem solution depends on the size of the linear system. In that case a static number (e.g. 50 iterations in the above example) is not appropriate. We can specify that the number of PCG iterations is half the size of the interface problem matrix, by replacing the corresponding line with
```csharp
interfaceProblemSolverBuilder.MaxIterationsProvider = new PercentageMaxIterationsProvider(0.5);
```

For problems where the stiffness distribution is not the same among subdomains, the following change is necessary:
```csharp
solverBuilder.ProblemIsHomogeneous = false;
```