function [Mayer,Lagrange]=leeBioreactorCost(solcost,setup);

t0 = solcost.initial.time;
s0 = solcost.initial.state;
tf = solcost.terminal.time;
sf = solcost.terminal.state;
t  = solcost.time;
s  = solcost.state;
u  = solcost.control;
p  = solcost.parameter;

Q = 0;

x4f = sf(4);
x1f = sf(1);
u2 = s(:,9);
u1dot = u(:,1);
u2dot = u(:,2);

f = 0.1/length(solcost.time);
Mayer = -(x4f*x1f);
% Lagrange = (Q*u2)+.003*((u1dot.^2)+(u2dot.^2));
Lagrange = (Q*u2)+f*((u1dot.^2)+(u2dot.^2));
