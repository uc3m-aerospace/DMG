function [Mayer,Lagrange,DMayer,DLagrange]=MaxradiusCost(sol,setup)

sigma0 = sol.initial.time;
x0 = sol.initial.state;
sigmaf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
p  = sol.parameter;

C1f    =xf(1);
C2f    =xf(2);
C3f    =xf(3);
tauf   =xf(4);
sigmaf= sigmaf*(p);

sf=1+C1f*cos(sigmaf)+C2f*sin(sigmaf);
rf = 1/(C3f^2*sf);
Mayer    = -rf;    % Maximize the orbital radius
Lagrange = zeros(size(t));
