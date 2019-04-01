function [C] = dmg_ObjandCons_coll(x)
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
global mysetup

C = constraints_coll(x,mysetup);
Cost= objective_coll(x,mysetup);

C=[Cost;C];


