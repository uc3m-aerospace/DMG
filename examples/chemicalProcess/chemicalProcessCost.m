function [Mayer,Lagrange,DMayer,DLagrange]=chemicalProcessCost(solcost,setup);

t0 = solcost.initial.time;
s0 = solcost.initial.state;
tf = solcost.terminal.time;
sf = solcost.terminal.state;
t  = solcost.time;
s  = solcost.state;
u  = solcost.control;
p  = solcost.parameter;

x = s(:,1);
z = s(:,2);

Mayer = zeros(size(t0));
Lagrange = x.^4+0.5.*z.^2+0.5.*u.^2;

if nargout == 4
    DMayer = [0 0 0 0 0 0];
    Lagrangex = 4*x.^3;
    Lagrangez = z;
    Lagrangeu = u;
    Lagranget = zeros(size(t));
    DLagrange = [Lagrangex Lagrangez Lagrangeu Lagranget];
end
