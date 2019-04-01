function [Mayer,Lagrange, DMayer, DLagrange] = launchCost(sol,setup);

global CONSTANTS

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;

Lagrange = zeros(size(t));
if sol.phase==4,
    Mayer = -xf(7);
else
    Mayer = zeros(size(t0));
end;

% avoid calc of derivs in not necessary
if nargout == 4
    
    if sol.phase
        % DMayer = [           dM/dx0,              dM/dt0,          dM/dxf, 
        DMayer = [zeros(1,length(x0)), zeros(1,length(t0)), [zeros(1,6) -1], ...
        ... %                  dM/dtf,              dM/dp]
                  zeros(1,length(tf)), zeros(1,length(p))];
    else
        % DMayer = [           dM/dx0,              dM/dt0,              dM/dxf, 
        DMayer = [zeros(1,length(x0)), zeros(1,length(t0)), zeros(1,length(xf)), ...
        ... %                  dM/dtf,              dM/dp]
                  zeros(1,length(tf)), zeros(1,length(p))]; 
    end

    % DLagrange = [       dL/dx,          dL/du,                      dL/dp,           dL/dt]
    DLagrange =[ zeros(size(x)), zeros(size(u)),  zeros(length(t),length(p)), zeros(size(t))];

end

