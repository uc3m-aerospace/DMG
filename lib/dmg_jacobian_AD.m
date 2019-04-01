function[J]=dmg_jacobian_AD(x,mysetup)
%------------------------------------------------------------------%
% Compute jacobian of constraints function for IPOPT when 
% Automatic differentiation is used             
%------------------------------------------------------------------%
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
% Unscale the decision variables
y=ad(x./mysetup.column_scales);
Jac= mysetup.constraints(y,mysetup);
J= sparse(getderivative(Jac)*mysetup.invDx);
J(1:mysetup.numnonlin,:) = mysetup.DF*J(1:mysetup.numnonlin,:);
J = sparse (J + mysetup.Alinear_augmented);


