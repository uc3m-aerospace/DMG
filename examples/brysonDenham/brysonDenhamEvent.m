%------------------------------------
% BEGIN: function brysonDenhamEvent.m
%------------------------------------
function [event,Devent] = brysonDenhamEvent(sol,setup);

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;

event(1:5,:) = [x0; xf(1:2)];

if nargout == 2
    Devent = [1 0 0 0 0 0 0 0;
              0 1 0 0 0 0 0 0;
              0 0 1 0 0 0 0 0;
              0 0 0 0 1 0 0 0;
              0 0 0 0 0 1 0 0];
end
%------------------------------------
% END: function brysonDenhamEvent.m
%------------------------------------
