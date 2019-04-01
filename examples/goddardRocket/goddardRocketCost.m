function [Mayer,Lagrange]=goddardRocketCost(sol,setup);

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
x  = sol.state;
u  = sol.control;
p  = sol.parameter;

if sol.phase==3,
    hf = xf(1);
    Mayer    = -hf; % Maximize the altitude
    Lagrange = zeros(size(t));
else
    Mayer    = zeros(size(tf));
    Lagrange = zeros(size(t));
end;    
