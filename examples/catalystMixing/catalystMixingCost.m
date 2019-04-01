function [Mayer,Lagrange] = catalystMixingCost(sol,setup)

if 0,
if ~isequal(sol.phase,3)
    Mayer    = zeros(size(sol.terminal.time));
else
    Mayer    = -1+sol.terminal.state(1)+sol.terminal.state(2);
end;
end;
Mayer    = -1+sol.terminal.state(1)+sol.terminal.state(2);
Lagrange = zeros(size(sol.time));
