function [C,G] = gpopsuserfunADMAD(x);
%------------------------------------------------------------------%
% User function when MATLAB automatic differentiation is used      %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup

%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = fmad(x./mysetup.column_scales,mysetup.seed);
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
CC = gpopsObjandCons(y);
%----------------------------------------------------------------%
% Get the values of the constraint vector and objective function %
%----------------------------------------------------------------%
C = getvalue(CC);
%-----------------------%
% Scale the constraints %
%-----------------------%
C(2:mysetup.numnonlin+1) = mysetup.DF*C(2:mysetup.numnonlin+1);
%------------------------------------------------------------%
% Get the Jacobian of the constraints and objective function %
%------------------------------------------------------------%
J = getinternalderivs(CC)*mysetup.invDx;
%----------------------%
% Unscale the Jacobian %
%----------------------%
J(2:mysetup.numnonlin+1,:) = mysetup.DF*J(2:mysetup.numnonlin+1,:);
G = snfindG(mysetup.iGfun,mysetup.jGvar,J);
