function [Mayer,Lagrange,DMayer,DLagrange]=MaxradiusCost(sol,setup);

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
x  = sol.state;
u  = sol.control;
p  = sol.parameter;

Mayer    = +0.001*tf + 0.999*sqrt(sum(x(:,2).^2+ x(:,3).^2)); % Maximize the orbital radius
Lagrange = zeros(size(t));
