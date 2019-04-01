function[G]=dmg_gradient_AD(x,mysetup)
%------------------------------------------------------------------%
% Compute gradient of objective function for IPOPT when 
% automatic differentiation is used             
%------------------------------------------------------------------%
% DMG Copyright   (c) David Morante González                       %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
y   = ad(x./mysetup.column_scales);
Obj = mysetup.objective(y,mysetup);
G   = sparse(getderivative(Obj)*mysetup.invDx);


