%-------------------------------------
% BEGIN: function hyperSensitiveCost.m
%-------------------------------------
function [Mayer,Lagrange,DMayer,DLagrange] = hyperSensitiveCost(sol);

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
x  = sol.state;
u  = sol.control;
p  = sol.parameter;
Mayer = zeros(size(t0));
Lagrange = 0.5*(x.^2+u.^2);

if nargout == 4
    % DMayer = [           dM/dx0,              dM/dt0,              dM/dxf, 
    DMayer = [zeros(1,length(x0)), zeros(1,length(t0)), zeros(1,length(xf)), ...
    ... %                  dM/dtf,              dM/dp]
             zeros(1,length(tf)), zeros(1,length(p))];

    % DLagrange = [ dL/dx, dL/du,                       dL/dp,          dL/dt]
    DLagrange =[        x,     u,  zeros(length(t),length(p)), zeros(size(t))];
end

%-------------------------------------
% END: function hyperSensitiveCost.m
%-------------------------------------