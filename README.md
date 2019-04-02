# DMG Tool
**D**irect **M**ethod based on **G**POPS.

This is a Matlab implementation of a General Purpose Software for solving Multi-phase Optimal Control Problems.

This distribution is a derived work from "GPOPS" (Gauss Pseudospectral Optimization Software).
The original GPOPS distribution can be downloaded from [here](https://es.mathworks.com/matlabcentral/fileexchange/21729-gpops). It has been developed at MIT, Draper Laboratory, and The University of Florida, and is published under the Simple Public License.

The main GPOPS features are:

 * Implements Gauss Pseudospectral Collocation Method for Transcribing the continuous optimal control problem into an Nonlinear Optimization Problem.
 * Interfaces with SNOPT
 * Includes Automatic, Forward Numerical, Complex-Step and Analytical Derivatives for gradient and jacobian computations.

The new features DMG includes:

 * Implements Hermite Simpson Collocation Method for Transcribing the continuous optimal control problem into a Nonlinear Optimization Problem.
 * Interfaces with IPOPT
 * Multi-core computation of Forward numerical derivatives and Complex Step differentiation.

### Goal
The purpose of DMG tool is to provide a Matlab open-source tool for solving Optimal Control Problems. Given that the original version of GPOPS is coded to work with the NLP solver SNOPT, which is a propietary software, we have modified the code in such a way that now is able to work with the open-source solver IPOPT.  

### NLP SOLVERS
Two different NLP solvers can be used with DMG solver:
 * [IPOPT](https://projects.coin-or.org/Ipopt): Interior Point Solver. It is an opensource package for solving large scale nonlinear optimization problems. Precompiled binaries for Macosx, Windows (32 and 64 bits), and Linux are included within the DMG solver distribution. They are located in *NLPsolvers/IPOPT*
 * [SNOPT](https://web.stanford.edu/group/SOL/snopt.htm): Sequential Quadratic Solver. It is a propietary software. A license can be purchased [here](https://ccom.ucsd.edu/~optimizers/downloads/). Copy and Paste the required libraries and mex files in the folder *NLPsolvers/SNOPT*


## Installation Guide
Installation requires simply to follow the next steps:
 * Download [DMG](https://github.com/uc3m-aerospace/DMG). 
 * Select the *DMG* directory as your working directory.
 * Run the *DMGSetup.m* file. It includes all the necessary paths to your root base directory. The file only need to be run once.

## Quick Usage Guide

In order to optimize an Optimal Control Problem, the user needs to call the main function *DMG.m* providing an input structure. Here you have an example:

```matlab
% INPUT STRUCTURE FOR DMG SOLVER
input.name        = 'ProblemName';      % name of the problem
input.funcs.cost  = 'CostFunctionName'; % name of the Cost Function
input.funcs.dae   = 'DaeFunction';      % name of the Differential Algebrais System
input.funcs.event = 'EventFunction';    % name of the Event function
input.funcs.link  = 'LinkFunction';     % name of the Link function
input.limits      = limits;             % Struture containing States, Controls, Parameters lower and Upper bounds as well as
                                        % Path and Event constraints Lower and Upper bounds.
input.guess       = guess;              % Struture containing initial guess for the States, Controls and Parameters
input.linkages    = linkages;           % Structure containing the desired values for the 'LinkFunction'
input.derivatives = 'automatic';        % Method for computing gradients and jacobians: automatic/numerical/complex/analytical
input.parallel    = 'no';               % Multicore computation of numerical and complex differentiation: yes/no
input.autoscale   = 'off';              % Automatic Scaling: yes/no
input.solver      = 'ipopt';            % NonLinear Programming Solver: ipopt/snopt
input.method      = 'collocation';      % Transcription method: collocation/pseudospectral

% RUN DMG TOOL
output  = DMG(input)
```
**NOTE**: Ensure that all the folders and subfolders are in the matlab path.
