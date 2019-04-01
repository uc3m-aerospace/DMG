function [Mayer Lagrange DMayer DLagrange] = gpopsConvertCost(solcost);
%------------------------------------------------------------------%
% Wrapper function to convert cost format from                     %
% Cell array to structures for backward compatibliity              %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup

sol{1,1} = solcost.initial.time;
sol{1,2} = solcost.initial.state;
sol{1,3} = solcost.terminal.time;
sol{1,4} = solcost.terminal.state;
sol{2,1} = solcost.time;
sol{2,2} = solcost.state;
sol{2,3} = solcost.control;
sol{2,4} = solcost.parameter;
    
iphase = solcost.phase;  

% call user cost function
if nargout == 2
    [Mayer, Lagrange] = feval(mysetup.funcsTemp{1},sol,iphase);
elseif nargout == 4
    [Mayer, Lagrange, DMayer, DLagrange] = feval(mysetup.funcsTemp{1},sol,iphase);
end