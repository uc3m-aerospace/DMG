% -----------------------------
% Linear Tangent Steering Example Problem
% -----------------------------

clear setup limits guess

x10 = 0;
x20 = 0;
x30 = 0;
x40 = 0;
x2f = 5;
x3f = 45;
x4f = 0;
x1min = -100;
x1max = 100;
x2min = x1min;
x2max = x1max;
x3min = x1min;
x3max = x1max;
x4min = x1min;
x4max = x1max;

param_min = [];
param_max = [];
duration_min = [];
duration_max = [];

iphase = 1;
limits(iphase).nodes            = 40;
limits(iphase).time.min         = [0 0];
limits(iphase).time.max         = [0 100];
limits(iphase).state.min(1,:)  = [x10 x1min x1min];
limits(iphase).state.max(1,:)  = [x10 x1max x1max];
limits(iphase).state.min(2,:)  = [x20 x2min x2f];
limits(iphase).state.max(2,:)  = [x20 x2max x2f];
limits(iphase).state.min(3,:)  = [x30 x3min x3f];
limits(iphase).state.max(3,:)  = [x30 x3max x3f];
limits(iphase).state.min(4,:)  = [x40 x3min x4f];
limits(iphase).state.max(4,:)  = [x40 x3max x4f];
limits(iphase).control.min     = -100;
limits(iphase).control.max     =  100;
limits(iphase).parameter.min   = [];
limits(iphase).parameter.max   = [];
limits(iphase).path.min        = [];
limits(iphase).path.max        = [];
limits(iphase).event.min       = [];
limits(iphase).event.max       = [];
limits(iphase).duration.min     = [];
limits(iphase).duration.max     = [];
guess(iphase).time              = [0; 0.5];
guess(iphase).state(:,1)       = [x10; x10];
guess(iphase).state(:,2)       = [x20; x2f];
guess(iphase).state(:,3)       = [x30; x3f];
guess(iphase).state(:,4)       = [x40; x4f];
guess(iphase).control          = [0; 0];
guess(iphase).parameter        = [];

setup.name  = 'Linear-Tangent-Steering-Problem';
setup.funcs.cost = 'linearTangentCost';
setup.funcs.dae = 'linearTangentDae';
setup.limits = limits;
setup.guess = guess;
setup.linkages = [];
setup.derivatives = 'automatic';
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='collocation';


output = DMG(setup);
solution = output.solution;

