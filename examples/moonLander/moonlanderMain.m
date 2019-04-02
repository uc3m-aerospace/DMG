clear setup limits guess CONSTANTS

global CONSTANTS

CONSTANTS.g = 1.6;

h0 = 10;
hf = 0;
v0 = -2;
vf = 0;

hmin = -20;
hmax =  20;
vmin = -20;
vmax =  20;
umin = -10;
umax =  10;
t0min = 0;
t0max = 0;
tfmin = 0;
tfmax = 1000;

% Phase 1 Information
iphase = 1;
limits(iphase).nodes           = 50;
limits(iphase).time.min        = [t0min tfmin];
limits(iphase).time.max        = [t0max tfmax];
limits(iphase).state.min(1,:) = [h0 hmin hf];
limits(iphase).state.max(1,:) = [h0 hmax hf];
limits(iphase).state.min(2,:) = [v0 vmin vf];
limits(iphase).state.max(2,:) = [v0 vmax vf];
limits(iphase).control.min    = umin;
limits(iphase).control.max    = umax;
limits(iphase).parameter.min  = [];
limits(iphase).parameter.max  = [];
limits(iphase).path.min       = [];
limits(iphase).path.max       = [];
limits(iphase).event.min      = [];
limits(iphase).event.max      = [];
limits(iphase).duration.min    = [];
limits(iphase).duration.max    = [];
guess(iphase).time             = [t0min; tfmax];
guess(iphase).state(:,1)      = [h0; h0];
guess(iphase).state(:,2)      = [v0; v0];
guess(iphase).control         = [umin; umin];
guess(iphase).parameter       = [];
%
linkages = [];
%
setup.CONSTANTS = CONSTANTS;
setup.name  = 'Moon-Lander-Problem';
setup.funcs.cost = 'moonlanderCost';
setup.funcs.dae = 'moonlanderDae';
setup.limits = limits;
setup.guess = guess;
setup.linkages = linkages;
setup.derivatives = 'automatic'; %numerical/complex/automatic
setup.parallel    = 'no';
setup.autoscale = 'off'; %on/off
setup.solver ='ipopt';% snopt /ipopt
setup.method ='hermite-simpson'; %pseudospectral/collocation

output = DMG(setup);


solution = output.solution;



