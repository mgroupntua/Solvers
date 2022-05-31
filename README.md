![alt text](http://mgroup.ntua.gr/wp-content/uploads/2018/05/MGroup52.png "MGroup")

# Solvers
This library is featuring linear algebra solvers for the solution of spatially and temporally discretized mechanics problems. In its core, it serves as a binding between the [LinearAlgebra](https://github.com/mgroupntua/linearalgebra) and the [MSolve.Core](https://github.com/mgroupntua/MSolve.Core) repos.

[![Build Status](https://dev.azure.com/mgroupntua/MSolve/_apis/build/status/mgroupntua.Solvers?branchName=develop)](https://dev.azure.com/mgroupntua/MSolve/_build/latest?definitionId=14&branchName=develop)

## Features

- **Assemblers:** Global matrix assemblers that take under consideration the connectivity of the underlying model supporting various formats including:
  * Dense
  * Compressed sparse row (CSR) (both symmetric and non-symmetric)
  * Skyline
  
- **Ordering:** Re-ordering strategies for the full matrices are supported for the minimization of matrix bandwidth 
  
- **Direct solvers:** Solvers that use factorization are implemented for dense, skyline and sparse formats

- **Iterative solvers:** Solvers that rely on "guessing" the solution by performing iterations are implemented, including the PCG and GMRES methods
  
- **Domain decomposition solvers:** Solvers that rely on the geometric and/or algebraic partitioning of the model at hand are implemented, including:
  * Overlapping methods: The Schwarz method is implemented
  * Dual methods: A series of variants of the FETI method are implemented

## Installation instructions
You can choose either to clone the solution or downloads it as a zip file.

### Clone solution
1. Under the repository name, click **Clone or Download** option.

![alt text](https://github.com/mgroupntua/MSolve.Edu/blob/master/Images/CloneOrDownload.png "1")

2. In the popup appearing choose the **Use HTTPS** option.

![alt text](https://github.com/mgroupntua/MSolve.Edu/blob/master/Images/2.png "2")

3. Use the ![alt text](https://github.com/mgroupntua/MSolve.Edu/blob/master/Images/3.png "3") to copy the link provided.

4. Open Visual Studio. In Team Explorer window appearing in your screen under Local Git Repositories click the **Clone** option. If Team Explorer window is not visible you can enable in View -> Team Explorer

  ![alt text](https://github.com/mgroupntua/MSolve.Edu/blob/master/Images/4.png "4")
  
5. In the text box appearing paste the link.

 ![alt text](https://github.com/mgroupntua/MSolve.Edu/blob/master/Images/5.png "5")

6. Click clone and Visual Studio will automatically download and import **MSolve.Solvers**


### Download as ZIP
1. Under the repository name, click **Clone or Download** option

![alt text](https://github.com/mgroupntua/MSolve.Edu/blob/master/Images/CloneOrDownload.png "1")

2. Click **Download ZIP** option. **MSolve.Solvers** will be downloaded as a ZIP file.

3. Extract the ZIP file to the folder of choice.

4. Double click on **MSolve.Solvers.sln** file to open the code with Visual Studio

