# Intro
FETI-1 or simply FETI (Finite Element Tearing & Interconnecting) is a DDM method introduced by Farhat et al. (1991). It is the first implementation of a family of FETI methods, which use Lagrange multipliers to describe the forces between the subdomains and define the interface problem in terms of those. The Lagrange multipliers are dual quantities (e.g. forces in elasticity problems, while displacements are primal), these methods are often called **dual**. Later, other dual DDMs were developed as simpler and in many cases more efficient, alternatives. 

Before continuing to the theory presented next, it is recommended to read the [general DDM chapter](ch3_ddm_solvers.md). For a more detailed description of FETI-1, see [the original publication](https://www.researchgate.net/publication/227701161_A_Method_of_Finite_Element_Tearing_and_Interconnecting_and_Its_Parallel_Solution_Algorithm).

# Theory

## Lagrange multipliers
<img src="img/langrange_multipliers.svg" alt="Lagrange multipliers" width="500"/>

In FETI-1 method, the domain is divided into disconnected subdomains as in the above figure and the continuity between them is retained by enforcing equal displacements for instances of the same boundary DOF (degree of freedom). E.g. if subdomains `s=1,2` share the same boundary dof `k`:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;1&space;&&space;-1&space;\end{bmatrix}&space;\cdot&space;\begin{bmatrix}&space;u_k^{(1)}&space;\\&space;u_k^{(2)}&space;\end{bmatrix}&space;=&space;0" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;1&space;&&space;-1&space;\end{bmatrix}&space;\cdot&space;\begin{bmatrix}&space;u_k^{(1)}&space;\\&space;u_k^{(2)}&space;\end{bmatrix}&space;=&space;0" title="\begin{bmatrix} 1 & -1 \end{bmatrix} \cdot \begin{bmatrix} u_k^{(1)} \\ u_k^{(2)} \end{bmatrix} = 0" /></a>

By gathering the 1, -1 and 0 coefficients in a signed boolean matrix **B**<sup>s</sup> for all boundary DOFs of a subdomain *s*, we get series of continuity equations:

<a href="https://www.codecogs.com/eqnedit.php?latex=\sum_{s=1}^{N_s}&space;\mathbf{B}^s&space;\cdot&space;\mathbf{u}^s&space;=&space;\mathbf{0}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\sum_{s=1}^{N_s}&space;\mathbf{B}^s&space;\cdot&space;\mathbf{u}^s&space;=&space;\mathbf{0}" title="\sum_{s=1}^{N_s} \mathbf{B}^s \cdot \mathbf{u}^s = \mathbf{0}" /></a>

To solve the initial linear system in the presence of these constraints, we apply Lagrange multipliers at boundary DOFs. These are dual quantities and can be viewed as forces, while displacements are primal quantities. The resulting linear system is written as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;\mathbf{K}^e&space;&&space;(\mathbf{B}^e)^{T}&space;\\&space;\mathbf{B}^e&space;&&space;\mathbf{0}&space;\end{bmatrix}&space;\cdot&space;\begin{bmatrix}&space;\mathbf{u}^e&space;\\&space;\mathbf{\lambda}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;\mathbf{f}^e&space;\\&space;\mathbf{0}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;\mathbf{K}^e&space;&&space;(\mathbf{B}^e)^{T}&space;\\&space;\mathbf{B}^e&space;&&space;\mathbf{0}&space;\end{bmatrix}&space;\cdot&space;\begin{bmatrix}&space;\mathbf{u}^e&space;\\&space;\mathbf{\lambda}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;\mathbf{f}^e&space;\\&space;\mathbf{0}&space;\end{bmatrix}" title="\begin{bmatrix} \mathbf{K}^e & (\mathbf{B}^e)^{T} \\ \mathbf{B}^e & \mathbf{0} \end{bmatrix} \cdot \begin{bmatrix} \mathbf{u}^e \\ \mathbf{\lambda} \end{bmatrix} = \begin{bmatrix} \mathbf{f}^e \\ \mathbf{0} \end{bmatrix}" /></a>

where the superscript <sup>e</sup> denotes a matrix/vector where each subdomain matrix/vector contribution (with superscript <sup>s</sup>) is placed one after the other, without any overlap.

## Floating subdomains

A subdomain of FETI-1 can be “floating”, meaning there are not enough constrained DOFs and its stiffness matrix **K**<sup>s</sup> is singular. To overcome this, we use a generalized inverse (**K**<sup>s</sup>)<sup>*</sup> and a normalized basis for its nullspace **R**<sup>s</sup>, which corresponds to the rigid body motions of the floating subdomain.

<a href="https://www.codecogs.com/eqnedit.php?latex=\left(&space;\mathbf{K}^{s}&space;\right)^{&plus;}&space;=&space;\begin{bmatrix}&space;\left(&space;\mathbf{K}_{11}^{s}&space;\right)^{-1}&space;&&space;\mathbf{0}&space;\\&space;\mathbf{0}&space;&&space;\mathbf{0}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\left(&space;\mathbf{K}^{s}&space;\right)^{&plus;}&space;=&space;\begin{bmatrix}&space;\left(&space;\mathbf{K}_{11}^{s}&space;\right)^{-1}&space;&&space;\mathbf{0}&space;\\&space;\mathbf{0}&space;&&space;\mathbf{0}&space;\end{bmatrix}" title="\left( \mathbf{K}^{s} \right)^{+} = \begin{bmatrix} \left( \mathbf{K}_{11}^{s} \right)^{-1} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} \end{bmatrix}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{R}^{s}&space;=&space;\begin{bmatrix}&space;\left(&space;\mathbf{K}_{11}^{s}&space;\right)^{-1}&space;\cdot&space;\mathbf{K}_{12}^s&space;\\&space;\mathbf{I}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{R}^{s}&space;=&space;\begin{bmatrix}&space;\left(&space;\mathbf{K}_{11}^{s}&space;\right)^{-1}&space;\cdot&space;\mathbf{K}_{12}^s&space;\\&space;\mathbf{I}&space;\end{bmatrix}" title="\mathbf{R}^{s} = \begin{bmatrix} \left( \mathbf{K}_{11}^{s} \right)^{-1} \cdot \mathbf{K}_{12}^s \\ \mathbf{I} \end{bmatrix}" /></a>

where the subscripts 1/2 denote linearly independent/dependent rows and columns, respectively. If the subdomain has sufficient constraints then **R**<sup>s</sup> is empty and (**K**<sup>s</sup>)<sup>*</sup> = (**K**<sup>s</sup>)<sup>-1</sup>. 

## Interface problem
The interface problem of FETI-1 is as follows:

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;\mathbf{F}_{I}&space;&&space;-\mathbf{G}_{I}&space;\\&space;-\mathbf{G}_{I}^{T}&space;&&space;\mathbf{0}&space;\end{bmatrix}&space;\cdot&space;\begin{bmatrix}&space;\mathbf{\lambda}&space;\\&space;\mathbf{a}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;\mathbf{d}&space;\\&space;-\mathbf{e}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;\mathbf{F}_{I}&space;&&space;-\mathbf{G}_{I}&space;\\&space;-\mathbf{G}_{I}^{T}&space;&&space;\mathbf{0}&space;\end{bmatrix}&space;\cdot&space;\begin{bmatrix}&space;\mathbf{\lambda}&space;\\&space;\mathbf{a}&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;\mathbf{d}&space;\\&space;-\mathbf{e}&space;\end{bmatrix}" title="\begin{bmatrix} \mathbf{F}_{I} & -\mathbf{G}_{I} \\ -\mathbf{G}_{I}^{T} & \mathbf{0} \end{bmatrix} \cdot \begin{bmatrix} \mathbf{\lambda} \\ \mathbf{a} \end{bmatrix} = \begin{bmatrix} \mathbf{d} \\ -\mathbf{e} \end{bmatrix}" /></a>

where **a** expresses linear combinations of the normalized rigid body motions in **R**, **F**<sub>I</sub> is the flexibility matrix, **G** aggregates the rigid body motions, **d** are the displacements due to applied forces (boundary conditions and loading) and **e** is the work of rigid body motions due to these applied forces:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{F}_{I}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{F}_{I}^{s}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}^{s}&space;\cdot&space;(\mathbf{K}^{s})^{&plus;}&space;\cdot&space;(\mathbf{B}^{s})^{T}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{F}_{I}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{F}_{I}^{s}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}^{s}&space;\cdot&space;(\mathbf{K}^{s})^{&plus;}&space;\cdot&space;(\mathbf{B}^{s})^{T}" title="\mathbf{F}_{I} = \sum_{s = 1}^{N_{s}} \mathbf{F}_{I}^{s} = \sum_{s = 1}^{N_{s}} \mathbf{B}^{s} \cdot (\mathbf{K}^{s})^{+} \cdot (\mathbf{B}^{s})^{T}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{d}&space;=&space;\sum_{s&space;=&space;1}^{d_{s}}&space;\mathbf{F}_{I}^{s}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}^{s}&space;\cdot&space;(\mathbf{K}^{s})^{&plus;}&space;\cdot&space;\mathbf{f}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{d}&space;=&space;\sum_{s&space;=&space;1}^{d_{s}}&space;\mathbf{F}_{I}^{s}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}^{s}&space;\cdot&space;(\mathbf{K}^{s})^{&plus;}&space;\cdot&space;\mathbf{f}^{s}" title="\mathbf{d} = \sum_{s = 1}^{d_{s}} \mathbf{F}_{I}^{s} = \sum_{s = 1}^{N_{s}} \mathbf{B}^{s} \cdot (\mathbf{K}^{s})^{+} \cdot \mathbf{f}^{s}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{G}_{I}&space;=&space;\begin{bmatrix}&space;\mathbf{B}^{(1)}&space;\cdot&space;\mathbf{R}^{(1)}&space;&&space;\mathbf{B}^{(2)}&space;\cdot&space;\mathbf{R}^{(2)}&space;&&space;\cdots&space;&&space;\mathbf{B}^{(N_s)}&space;\cdot&space;\mathbf{R}^{(N_s)}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{G}_{I}&space;=&space;\begin{bmatrix}&space;\mathbf{B}^{(1)}&space;\cdot&space;\mathbf{R}^{(1)}&space;&&space;\mathbf{B}^{(2)}&space;\cdot&space;\mathbf{R}^{(2)}&space;&&space;\cdots&space;&&space;\mathbf{B}^{(N_s)}&space;\cdot&space;\mathbf{R}^{(N_s)}&space;\end{bmatrix}" title="\mathbf{G}_{I} = \begin{bmatrix} \mathbf{B}^{(1)} \cdot \mathbf{R}^{(1)} & \mathbf{B}^{(2)} \cdot \mathbf{R}^{(2)} & \cdots & \mathbf{B}^{(N_s)} \cdot \mathbf{R}^{(N_s)} \end{bmatrix}" /></a>

The linear system written above is not symmetric positive definite. To solve it using the preconditioned conjugate gradient (PCG) method, we must project it into another space using a projection matrix **P** and then solve the final linear system of the interface problem:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{P}^{T}\mathbf{F}_{I}\mathbf{P}\mathbf{\bar{\lambda}}&space;=&space;\mathbf{P}^{T}&space;(\mathbf{d}&space;-&space;\mathbf{F}_{I}\mathbf{P}\mathbf{\lambda_{0}})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{P}^{T}\mathbf{F}_{I}\mathbf{P}\mathbf{\bar{\lambda}}&space;=&space;\mathbf{P}^{T}&space;(\mathbf{d}&space;-&space;\mathbf{F}_{I}\mathbf{P}\mathbf{\lambda_{0}})" title="\mathbf{P}^{T}\mathbf{F}_{I}\mathbf{P}\mathbf{\bar{\lambda}} = \mathbf{P}^{T} (\mathbf{d} - \mathbf{F}_{I}\mathbf{P}\mathbf{\lambda_{0}})" /></a>

where

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{P}&space;=&space;\mathbf{I}&space;-&space;\mathbf{G}_{I}(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}\mathbf{G}_{I}^{T}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{P}&space;=&space;\mathbf{I}&space;-&space;\mathbf{G}_{I}(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}\mathbf{G}_{I}^{T}" title="\mathbf{P} = \mathbf{I} - \mathbf{G}_{I}(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}\mathbf{G}_{I}^{T}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{\lambda}_{0}&space;=&space;\mathbf{G}_{I}&space;(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}\mathbf{e}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{\lambda}_{0}&space;=&space;\mathbf{G}_{I}&space;(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}\mathbf{e}" title="\mathbf{\lambda}_{0} = \mathbf{G}_{I} (\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}\mathbf{e}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{\lambda}&space;=&space;\mathbf{\lambda}_{0}&space;&plus;&space;\mathbf{P}&space;\bar{\mathbf{\lambda}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{\lambda}&space;=&space;\mathbf{\lambda}_{0}&space;&plus;&space;\mathbf{P}&space;\bar{\mathbf{\lambda}}" title="\mathbf{\lambda} = \mathbf{\lambda}_{0} + \mathbf{P} \bar{\mathbf{\lambda}}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{a}&space;=&space;(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}&space;\mathbf{G}_{I}^{T}&space;(\mathbf{F}_I\mathbf{\lambda}&space;-&space;\mathbf{d})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{a}&space;=&space;(\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1}&space;\mathbf{G}_{I}^{T}&space;(\mathbf{F}_I\mathbf{\lambda}&space;-&space;\mathbf{d})" title="\mathbf{a} = (\mathbf{G}_{I}^{T}\mathbf{G}_{I})^{-1} \mathbf{G}_{I}^{T} (\mathbf{F}_I\mathbf{\lambda} - \mathbf{d})" /></a>

Finally we calculate the displacements depending on if the subdomain is floating or not:

<<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{u}^{s}&space;=&space;(\mathbf{K}^{s})^{&plus;}&space;(\mathbf{f}^{s}&space;-&space;(\mathbf{B}^{s})^{T}&space;\mathbf{\lambda})&space;&plus;&space;\mathbf{R}^{s}\mathbf{a}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{u}^{s}&space;=&space;(\mathbf{K}^{s})^{&plus;}&space;(\mathbf{f}^{s}&space;-&space;(\mathbf{B}^{s})^{T}&space;\mathbf{\lambda})&space;&plus;&space;\mathbf{R}^{s}\mathbf{a}^{s}" title="\mathbf{u}^{s} = (\mathbf{K}^{s})^{+} (\mathbf{f}^{s} - (\mathbf{B}^{s})^{T} \mathbf{\lambda}) + \mathbf{R}^{s}\mathbf{a}^{s}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{u}^{s}&space;=&space;(\mathbf{K}^{s})^{-1}&space;(\mathbf{f}^{s}&space;-&space;(\mathbf{B}^{s})^{T}&space;\mathbf{\lambda})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{u}^{s}&space;=&space;(\mathbf{K}^{s})^{-1}&space;(\mathbf{f}^{s}&space;-&space;(\mathbf{B}^{s})^{T}&space;\mathbf{\lambda})" title="\mathbf{u}^{s} = (\mathbf{K}^{s})^{-1} (\mathbf{f}^{s} - (\mathbf{B}^{s})^{T} \mathbf{\lambda})" /></a>

## Coarse problem
The coarse problem of FETI-1 is expressed in the linear system <a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{G}_{I}^{T}&space;\mathbf{G}_{I}&space;\cdot&space;\mathbf{x}&space;=&space;\mathbf{y}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{G}_{I}^{T}&space;\mathbf{G}_{I}&space;\cdot&space;\mathbf{x}&space;=&space;\mathbf{y}" title="\mathbf{G}_{I}^{T} \mathbf{G}_{I} \cdot \mathbf{x} = \mathbf{y}" /></a> contained in the interface problem system above. It helps speed up convergence by globally distributing the error at each PCG iteration. 

## Preconditioning
FETI-1 uses a standard preconditioner to speed up the convergence of PCG when solving the interface problem. The idea is to approximate the inverse of the flexibility matrix, which is defined for the boundary DOFs. The most common preconditioners used are Dirichlet and lumped.

### Dirichlet preconditioner
Dirichlet preconditioner is defined as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{F}_{I}^{-1}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}_{pb}^{s}&space;\mathbf{S}^{s}&space;(\mathbf{B}_{pb}^{s})^{T}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{F}_{I}^{-1}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}_{pb}^{s}&space;\mathbf{S}^{s}&space;(\mathbf{B}_{pb}^{s})^{T}" title="\mathbf{F}_{I}^{-1} = \sum_{s = 1}^{N_{s}} \mathbf{B}_{pb}^{s} \mathbf{S}^{s} (\mathbf{B}_{pb}^{s})^{T}" /></a>

where **S**<sup>s</sup> is the Schur complement of internal dofs

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{S}^{s}&space;=&space;\mathbf{K}_{bb}^{s}&space;-&space;(\mathbf{K}_{ib}^{s})^{T}(\mathbf{K}_{ii}^{s})^{-1}\mathbf{K}_{ib}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{S}^{s}&space;=&space;\mathbf{K}_{bb}^{s}&space;-&space;(\mathbf{K}_{ib}^{s})^{T}(\mathbf{K}_{ii}^{s})^{-1}\mathbf{K}_{ib}^{s}" title="\mathbf{S}^{s} = \mathbf{K}_{bb}^{s} - (\mathbf{K}_{ib}^{s})^{T}(\mathbf{K}_{ii}^{s})^{-1}\mathbf{K}_{ib}^{s}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{B}_{pb}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{B}_{pb}^{s}" title="\mathbf{B}_{pb}^{s}" /></a> are sparse matrices that perform mapping (like **B**<sup>s</sup>) and scaling. E.g. for a homogeneous stiffness distribution among subdomains:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{B}_{pb}^{s}&space;=&space;\mathbf{B}_{b}^{s}(\mathbf{M}_{b}^{s})^{-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{B}_{pb}^{s}&space;=&space;\mathbf{B}_{b}^{s}(\mathbf{M}_{b}^{s})^{-1}" title="\mathbf{B}_{pb}^{s} = \mathbf{B}_{b}^{s}(\mathbf{M}_{b}^{s})^{-1}" /></a>

where <a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{B}_{b}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{B}_{b}^{s}" title="\mathbf{B}_{b}^{s}" /></a> are the columns of <a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{B}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{B}^{s}" title="\mathbf{B}^{s}" /></a> corresponding to boundary DOFs (boundary remainder in FETI-DP) and <a href="https://www.codecogs.com/eqnedit.php?latex=$\mathbf{M}_{b}$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$\mathbf{M}_{b}$" title="$\mathbf{M}_{b}$" /></a> is a diagonal matrix whose diagonal entries are the multiplicity of the corresponding DOFs, i.e. how many subdomains have instances of the DOFs.

### Lumped preconditioner
Lumped preconditioner is defined as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{F}_{I}^{-1}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}_{pb}^{s}&space;\mathbf{K}_{bb}^{s}&space;(\mathbf{B}_{pb}^{s})^{T}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{F}_{I}^{-1}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}_{pb}^{s}&space;\mathbf{K}_{bb}^{s}&space;(\mathbf{B}_{pb}^{s})^{T}" title="\mathbf{F}_{I}^{-1} = \sum_{s = 1}^{N_{s}} \mathbf{B}_{pb}^{s} \mathbf{K}_{bb}^{s} (\mathbf{B}_{pb}^{s})^{T}" /></a>

In general, lumped preconditioners are faster to compute and implement, but does not speed up the convergence of PCG as much as Dirichlet.

### Diagonal Dirichlet preconditioner
Diagonal Dirichlet preconditioner is defined as:

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{F}_{I}^{-1}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}_{pb}^{s}&space;\mathbf{\hat{S}}^{s}&space;(\mathbf{B}_{pb}^{s})^{T}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{F}_{I}^{-1}&space;=&space;\sum_{s&space;=&space;1}^{N_{s}}&space;\mathbf{B}_{pb}^{s}&space;\mathbf{\hat{S}}^{s}&space;(\mathbf{B}_{pb}^{s})^{T}" title="\mathbf{F}_{I}^{-1} = \sum_{s = 1}^{N_{s}} \mathbf{B}_{pb}^{s} \mathbf{\hat{S}}^{s} (\mathbf{B}_{pb}^{s})^{T}" /></a>

where <a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{\hat{S}}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{\hat{S}}^{s}" title="\mathbf{\hat{S}}^{s}" /></a> is an approximation of the Schur complement of internal dofs. 

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{\hat{S}}^{s}&space;=&space;\mathbf{K}_{bb}^{s}&space;-&space;(\mathbf{K}_{ib}^{s})^{T}(\mathbf{D}_{ii}^{s})^{-1}\mathbf{K}_{ib}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{\hat{S}}^{s}&space;=&space;\mathbf{K}_{bb}^{s}&space;-&space;(\mathbf{K}_{ib}^{s})^{T}(\mathbf{D}_{ii}^{s})^{-1}\mathbf{K}_{ib}^{s}" title="\mathbf{\hat{S}}^{s} = \mathbf{K}_{bb}^{s} - (\mathbf{K}_{ib}^{s})^{T}(\mathbf{D}_{ii}^{s})^{-1}\mathbf{K}_{ib}^{s}" /></a>

where <a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{D}_{ii}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{D}_{ii}^{s}" title="\mathbf{D}_{ii}^{s}" /></a> is the diagonal of <a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{K}_{ii}^{s}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{K}_{ii}^{s}" title="\mathbf{K}_{ii}^{s}" /></a>. 

In general, the computational cost of computing and implementing a diagonal Dirichlet 
preconditioner, as well as the resulting reduction of PCG iterations falls 
between that of Dirichlet and lumped preconditioner.

# Remarks
FETI-1 is an efficient DDM that exhibits good scalability, namely the iterations required to converge do not increase as the number of subdomain increases. This makes it applicable to large scale simulations run on computing systems with a lot of networked computers. Its disadvantages are caused by the existence of floating subdomains, the processing of which is difficult and very sensitive to accuracy loss. Floating subdomains and rigid body motions also cause problems when FETI-1 is applied to dynamic problems.