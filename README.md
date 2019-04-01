# DMG SOLVER
**D**irect **M**ethod based on **G**POPS.

This is a Matlab implementation of a General Purpose Software for solving Multi-phase Optimal Control Problems.

This distribution is a derived work from "GPOPS" (Gauss Pseudospectral Optimization Software).
The original GPOPS distribution can be downloaded from [here](https://es.mathworks.com/matlabcentral/fileexchange/21729-gpops) and is published under the Simple Public License.

The main GPOPS features are:

 * Implements Gauss Pseudospectral Collocation Method for Transcribing the continuous optimal control problem into an Nonlinear Optimization Problem.
 * Interfaces with SNOPT
 * Includes Automatic, Forward Numerical, Complex-Step and Analytical Derivatives for gradient and jacobian computations.

The new features DMG includes:

 * Implements Hermite Simpson Collocation Method for Transcribing the continuous optimal control problem into an Nonlinear Optimization Problem.
 * Interfaces with IPOPT
 * Multi-core computation of Forward numerical derivatives.
