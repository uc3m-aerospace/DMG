function [C,G] = gpopsuserfunAN(x,setup);
%------------------------------------------------------------------%
% User function when analytic differentiation is used             %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup

%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
%----------------------------------------------------------------%
% Get the values of the constraint vector and objective function %
%     and the Jacobian of the constraints and objective function %
%----------------------------------------------------------------%
[C, J] = gpopsObjandCons(y);
%-----------------------%
% Scale the constraints %
%-----------------------%
C(2:mysetup.numnonlin+1) = mysetup.DF*C(2:mysetup.numnonlin+1);
%----------------------%
% Scale the Jacobian   %
%----------------------%
J = J*mysetup.invDx;
J(2:mysetup.numnonlin+1,:) = mysetup.DF*J(2:mysetup.numnonlin+1,:);
G = snfindG(mysetup.iGfun,mysetup.jGvar,J);

