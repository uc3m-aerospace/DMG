% -----------------------------
% Bryson-Denham Example Problem
% -----------------------------
% --------------------------------------------------
% This example is taken from the following reference:
% --------------------------------------------------
% Bryson, A. E., Denham, W. F., and Dreyfus, S. E., "Optimal
% Programming Problems with Inequality Constraints.  I: Necessary
% Conditions for Extremal Solutions, AIAA Journal, Vol. 1, No. 11,
% November, 1963, pp. 2544-2550.
clear setup limits guess

x10 = 0;
x20 = 1;
x30 = 0;
x1f = 0;
x2f = -1;
x1min = -10;
x1max = 10;
x2min = x1min;
x2max = x1max;
x3min = x1min;
x3max = x1max;

param_min = [];
param_max = [];
path_min = 0;
path_max = 1/9;
event_min = [x10; x20; x30; x1f; x2f];
event_max = [x10; x20; x30; x1f; x2f];
duration_min = [];
duration_max = [];

iphase = 1;
limits(iphase).nodes = 50;
limits(iphase).time.min = [0 0];
limits(iphase).time.max = [0 50];
limits(iphase).state.min(1,:) = [x1min x1min x1min];
limits(iphase).state.max(1,:) = [x1max x1max x1max];
limits(iphase).state.min(2,:) = [x2min x2min x2min];
limits(iphase).state.max(2,:) = [x2max x2max x2max];
limits(iphase).state.min(3,:) = [x3min x3min x3min];
limits(iphase).state.max(3,:) = [x3max x3max x3max];
limits(iphase).control.min    = -5000;
limits(iphase).control.max    =  5000;
limits(iphase).parameter.min  = param_min;
limits(iphase).parameter.max  = param_max;
limits(iphase).path.min       = path_min;
limits(iphase).path.max       = path_max;
limits(iphase).event.min      = event_min;
limits(iphase).event.max      = event_max;
limits(iphase).duration.min   = [];
limits(iphase).duration.max   = [];
guess(iphase).time            = [0; 0.5];
guess(iphase).state(:,1)      = [x10; x1f];
guess(iphase).state(:,2)      = [x20; x2f];
guess(iphase).state(:,3)      = [x30; x30];
guess(iphase).control         = [0; 0];
guess(iphase).parameter       = [];

clear x10 x20 x30 x1f x2f x1min x1max x2min x2max x3min x3max 
clear param_min param_max path_min path_max event_min event_max
clear duration_min duration_max iphase

setup.name  = 'Bryson-Denham-Problem';
setup.funcs.cost = 'brysonDenhamCost';
setup.funcs.dae = 'brysonDenhamDae';
setup.funcs.event = 'brysonDenhamEvent';
setup.limits = limits;
setup.guess = guess;
setup.linkages = [];
setup.derivatives = 'analytic';
setup.parallel    = 'no';
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='collocation';

output = DMG(setup);
solution = output.solution;
%------------------------------------
% END: script brysonDenhamMain.m
%------------------------------------

