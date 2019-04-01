function [connect Dconnect] = gpopsConvertLinkage(sollink);
%------------------------------------------------------------------%
% Wrapper function to convert linkage format from                  %
% Cell array to structures for backward compatibliity              %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global mysetup
                
sol{1} = sollink.left.state;
sol{2} = sollink.left.parameter;
sol{3} = sollink.right.state;
sol{4} = sollink.right.parameter;
        
phaseL = sollink.left.phase;
phaseR = sollink.right.phase;

% call user linkage function
if nargout == 1
    connect = feval(mysetup.funcsTemp{4},sol,phaseL,phaseR);
elseif nargout == 2
    [connect Dconnect] = feval(mysetup.funcsTemp{4},sol,phaseL,phaseR);
end