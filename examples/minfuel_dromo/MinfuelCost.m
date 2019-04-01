function [Mayer,Lagrange,DMayer,DLagrange]=MinfuelCost(sol,setup)

p  = sol.parameter(1);
t = sol.time;
Mayer    = p;    % Maximize the orbital radius
Lagrange = zeros(size(t));
