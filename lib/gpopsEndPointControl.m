function F=gpopsEndPointControl(u,setup);
%------------------------------------------------------------------%
% Computes the Hamiltonian and the path constraints in order to    %
% compute the endpoint control for multiple-phase optimal control  %
% problem.  The endpoint control algorithm is given in the         %
% following reference:                                             %
%   Huntington, G. T., Benson, D. A., How, J. P., Kanizay, N.,     %
%   Darby, C. L., and Rao, A. V., "Computation of Endpoint Controls%
%   Using a Gauss Pseudospectral Method," 2007 Astrodynamics       %
%   Specialist Conference, Mackinac Island, Michigan,              %
%   August 19-23, 2007.                                            %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
global extras funcs

t      = extras.time;
x      = extras.state;
p      = extras.parameter;
lambda = extras.costate;
iphase = extras.phase;

nstates = length(x);
solcost.initial.time = t;
solcost.initial.state = x.';
solcost.terminal.time = t;
solcost.terminal.state = x.';
solcost.time = t;
solcost.state = x;
solcost.control = u.';
solcost.parameter = p;
solcost.phase = iphase;
soldae.time = t;
soldae.state = x;
soldae.control = u.';
soldae.parameter = p;
soldae.phase = iphase;
[Mayer,Lagrangian]=feval(funcs.cost,solcost,setup);
dae = feval(funcs.dae,soldae,setup);
xdot = dae(1:nstates).';
path = dae(nstates+1:end).';
H = Lagrangian+lambda*xdot;
F = [H; path];
