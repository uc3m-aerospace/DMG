function [Mayer,Lagrange,DMayer,DLagrange]=brysonMaxrangeCost(sol,setup);

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
x  = sol.state;
u  = sol.control;
p  = sol.parameter;

Mayer    = -xf(1); % Maximize the horizontal range
Lagrange = zeros(size(t));

if nargout == 4
    DMayer = [0 0 0 0 -1 0 0 0];
    DLagrange = [zeros(size(t)) zeros(size(t)) zeros(size(t)) zeros(size(t)) zeros(size(t)) zeros(size(t))];
end