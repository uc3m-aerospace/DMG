%------------------------------------------------------------------%
%              Initialize Integer Indices in Workspace             %
%------------------------------------------------------------------%

%------------------------------------------------------------------%
% Table of Integers:
%   imin       = Lower Bound
%   imax       = Upper Bound
%   itime      = Time Index
%   istate     = State Index
%   icontrol   = Control Index
%   iparameter = Static Parameter Index
%   ipath      = Path Constraint Index
%   ievent     = Event Constraint Index
%   iduration  = Phase Duration Index
%   icostate   = Costate Index
%   imultip    = Path Constraint Multipliers
%   ihamilt    = Hamiltonian Index
%   ilagrange  = Lagrange Cost Index
%   imayer     = Mayer Cost Index
%   iconnect   = Phase-Connect Index
%   fcost      = Cost Functional Index
%   fdae       = Differential-Algebraic Equation Function Index
%   fevent     = Event Function Index
%   fconnect   = Phase-Connect Function Index
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

imin = 1; imax = 2;
itime =1; istate = 2; icontrol = 3; iparameter = 4; ipath = 5;
ievent = 6; iduration = 7;
icostate = 5; imultip = 6; ihamilt = 7; imayer = 8; ilagrange = 9; 
iconnect = 3;
fcost = 1; fdae = 2; fevent = 3; fconnect = 4;

