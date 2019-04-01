function [event Devent] = gpopsConvertEvent(solevent);
%------------------------------------------------------------------%
% Wrapper function to convert event format from                    %
% Cell array to structures for backward compatibliity              %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup
          
sol{1} = solevent.initial.time;
sol{2} = solevent.initial.state;
sol{3} = solevent.terminal.time;
sol{4} = solevent.terminal.state;
sol{5} = solevent.parameter;
           
iphase = solevent.phase;  

% call user event function
if nargout == 1
    [event] = feval(mysetup.funcsTemp{3},sol,iphase);
elseif nargout == 2
    [event Devent] = feval(mysetup.funcsTemp{3},sol,iphase);
end