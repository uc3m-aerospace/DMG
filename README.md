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
 * Multi-core computation of Forward numerical derivatives and Complex Step differentiation.

### Goal
The purpose of MOLTO-IT is to provide a fast and robust mission design environment that allows the user to quickly and inexpensively perform trade studies of various mission configurations and conduct low-fidelity analysis. 

### NLP SOLVERS
Two different NLP solvers can be used with DMG solver:
 * [IPOPT](https://projects.coin-or.org/Ipopt): Interior Point Solver. It is an opensource package for solving large scale nonlinear optimization problems. Precompiled binaries for Macosx, Windows (32 and 64 bits), and Linux are included within the DMG solver distribution. They are located in *NLPsolvers/IPOPT*
 * [SNOPT](https://web.stanford.edu/group/SOL/snopt.htm): Sequential Quadratic Solver. It is a propietary software. A license can be purchased [here](https://ccom.ucsd.edu/~optimizers/downloads/). Copy and Paste the required libraries and mex files in the folder *NLPsolvers/SNOPT*


## Installation Guide
Installation requires simply that you download [DMG](https://github.com/uc3m-aerospace/DMG). Then, you have the *DMGSetup.m* file. It includes all the necessary path to your root base directory. The file only need to be run once.

Make sure that the *DMG* directory is your working directory.

## Quick Usage Guide

In order to optimize a mission, the user needs to call the main function *molto_it.m* providing an input structure. Here you vae an example:

```matlab
% INPUT STRUCTURE FOR DMG SOLVER
input.name        = 'ProblemName';      % name of the problem
input.funcs.cost  = 'CostFunctionName'; % name of the Cost Function
input.funcs.dae   = 'DaeFunction';      % name of the Differential Algebrais System
input.funcs.event = 'EventFunction';    % name of the Event function
input.funcs.link  = 'LinkFUnction';     % name of the Link function
input.limits      = limits;             % Struture containing limits
input.guess       = guess;              % Structure containing Initial Guess
input.linkages    = linkages;           % Structure containing Linkages Values
input.derivatives = 'automatic';        % Method for computing gradients and jacobians: automatic/numerical/complex/analytical
input.parallel    = 'no';               % Multicore computation of numerical and complex differentiation: yes/no
input.autoscale   = 'off';              % Automatic Scaling: yes/no
input.solver      = 'ipopt';            % NonLinear Programming Solver: ipopr/snopt
input.method      = 'collocation';      % Transcription method: collocation/pseudospectral

% RUN DMG SOLVER
output  = DMG(input)
```
**NOTE**: Ensure that all the folders and subfolders are in the matlab path.
