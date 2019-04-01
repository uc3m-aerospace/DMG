%------------------------------------
% BEGIN: function hyperSensitiveDae.m
%------------------------------------
function [dae Ddae] = hyperSensitiveDae(sol);

t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;

dae = -x.^3+u;

if nargout == 2
    % Ddae = [df/dx,         df/du,                      df/dp,          df/dt]  
    Ddae = [-3*x.^2, ones(size(u)), zeros(length(t),length(p)), zeros(size(t))];
end

%-----------------------------------
% END: function hyperSensitiveDae.m
%-----------------------------------