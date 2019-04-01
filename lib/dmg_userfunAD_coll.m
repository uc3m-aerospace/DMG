function [C] = gpopsObjandCons(x)
%------------------------------------------------------------------%
% User function when automatic differentiation is used             %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup


C = constraints_coll(y);
Cost= objective_coll(y);

C=[Cost;C];


