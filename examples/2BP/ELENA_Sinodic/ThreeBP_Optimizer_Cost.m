function [Mayer,Lagrange,DMayer,DLagrange]=TwoBP_Optimizer_Cost(sol,setup);

global constants
t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
t  = sol.time;
x  = sol.state;
u  = sol.control;
%m_0  = sol.parameter;
m = x(:,5);

Mayer    = t(end)*constants.tc/(3600*24); %zeros(size(t0)) %Mayer siempre intenta minimizar. Para maximizar cambiale el signo %Esto es para decirle que quieres maximizar el elemnto final
Lagrange = zeros(size(t)); %-m;

%if nargout == 4
%    DMayer = [0 0 0 0 -1 0 0 0];
%    DLagrange = [zeros(size(t)) zeros(size(t)) zeros(size(t)) zeros(size(t)) zeros(size(t)) zeros(size(t))];
%end