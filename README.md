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

## Installation Guide
Installation requires simply that you download [MOLTO-IT](https://github.com/uc3m-aerospace/MOLTO-IT/) and add the base directory to your Matlab path.

## Quick Usage Guide

In order to optimize a mission, the user needs to call the main function *molto_it.m* providing an input structure. Here you vae an example:

```matlab
% FLYBY MISSION TO JUPITER WITH UP TO 3 FLYBYS
        input.problem_name  = 'example'; % Problem name
        input.problem_type  = 'flyby'; % Type of mission: condition at arrival planet (flyby/rendezvous)
        input.planet_dep    = '3';     % Departure planet using space nomenclature (e.g. 3==Earth)
        input.planet_arr    = '5';     % Arrival planet using space nomenclature (e.g. 5==Jupiter)
        input.vinf0_max     =  2;      % Hyperbolic excess velocity at departure planet (km/s)
        input.planet_fb     = [{'4'},{'3'},{'2'}]; % List of available planets to flyby in spice nomenclature
        input.rfb_min       = 200;     % minimum flyby altitude (km)
        input.n_fb          = [0,3];   % minimum/maximum number of possible flybys
        input.rev           = [0,0];   % minimum/maximum number of possible revolutions
        input.ToF           = [50  50  50  50;  % minimum/maximum transfer time per leg (days)
                          500 500 500 1000];
        input.Initial_Date  = [{'2029 Jan 01 00:00:00'},{'2030 Dec 31 00:00:00'}]; % minimum/maximum Launch date (Gregorian Date)
        input.init_file     = [];      % Init population File name (if not provided, random initial population)
        input.output_file   = ['example','.txt']; % Solution population File name
        input.plot          = 0;       % plotting option (recomended = 0, option =1 is under development)
        input.useParallel   = 'yes';   % yes/no for parallel execution of the genetic algorithm
        input.maxGen        = 200;     % maximum number of generations
        input.popsize       = 200;     % Population Size
        input.spice_dir     =  '/home/MOLTO-IT/spice' % The spice directory folder
        
% RUN MOLTO-IT ALGORITHM
        molto_it(input)
```
**NOTE**: Ensure that all the folders and subfolders are in the matlab path.
