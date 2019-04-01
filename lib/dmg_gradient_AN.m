function[G]=dmg_gradient_AN(x,mysetup)
%------------------------------------------------------------------%
% Compute gradient of objective function for IPOPT when Analytic
% derivatives are provided is used             
%------------------------------------------------------------------%
% DMG Copyright   (c) David Morante González                       %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
y   = x./mysetup.column_scales;
[Obj,G] = mysetup.objective(y,mysetup);
G   = sparse(G*mysetup.invDx);