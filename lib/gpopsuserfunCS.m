function [C,G] = gpopsuserfunCS(x,setup);
%------------------------------------------------------------------%
% User function when complex step differentiation is used          %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

  global mysetup

%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
C = mysetup.gpopsObjandCons(y);
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
C(2:mysetup.numnonlin+1) = mysetup.DF*C(2:mysetup.numnonlin+1);
J = mysetup.Jaczeros;

for k=1:mysetup.numvars;
    ypert = y+mysetup.deltaxmat(:,k);
    J(:,k) = mysetup.gpopsObjandCons(ypert);
end;

J = imag(J/mysetup.hpert)*mysetup.invDx;
%----------------------%
% Unscale the Jacobian %
%----------------------%
J(2:mysetup.numnonlin+1,:) = mysetup.DF*J(2:mysetup.numnonlin+1,:);
G = snfindG(mysetup.iGfun,mysetup.jGvar,J);

