function [Mayer,Lagrange]=InterplanetaryCost(sol,setup)

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
x  = sol.state;
u  = sol.control;
p  = sol.parameter;

mf   = xf(7);

if sol.phase==setup.n_phases
    %
    % Masa Final
    %
    Mayer = -launcher_model(setup.launcher,p(end-1)^2)*(1.1*mf-0.1)/1000000;%
    Lagrange = zeros(size(t));
    
else
    
    Mayer    = 0; %
    %Lagrange = T.*Tmax./m;
    Lagrange = zeros(size(t));
    
end