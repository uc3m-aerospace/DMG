function [dae Ddae] = gpopsConvertDae(soldae);
%------------------------------------------------------------------%
% Wrapper function to convert dae format from                      %
% Cell array to structures for backward compatibliity              %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup

sol{1} = soldae.time;
sol{2} = soldae.state;
sol{3} = soldae.control;
sol{4} = soldae.parameter;
    
iphase = soldae.phase;  

% call user dae function
if nargout == 1
    [dae] = feval(mysetup.funcsTemp{2},sol,iphase);
elseif nargout == 2
    [dae, Ddae] = feval(mysetup.funcsTemp{2},sol,iphase);
end