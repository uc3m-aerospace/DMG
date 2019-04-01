% ----------------------------
% Bryson Maximum-Radius Example
% ----------------------------
% This problem is taken verbatim from the following reference:
% Bryson, A. E. and Ho, Y-C, Applied Optimal Control, Hemisphere
% Publishing, New York, 1975.
% ----------------------------

clear all
format long

global constants
constants.p1 = 1/0.1405;
constants.p2 = 0.5328825;

%%
r0 = 1;
u0 = 0;
v0 = 1;
rf = 1.52;
uf = 0;
vf= sqrt(1/rf);
rmin = 1;
rmax = 2;
umin = 0;
umax = 5;
vmin = -10;
vmax = -vmin;
u1min = -5;
u1max = -u1min;
u2min = u1min;
u2max = u1max;
t0 = 0;
tf = 3.32;
tmin=0.1;
tmax=6;

%%
iphase = 1;
limits(iphase).nodes =30;
limits(iphase).time.min = [t0 tmin];
limits(iphase).time.max = [t0 tmax];
limits(iphase).state.min(1,:)   = [r0 rmin rf];
limits(iphase).state.max(1,:)   = [r0 rmax rf];
limits(iphase).state.min(2,:)   = [u0 umin uf];
limits(iphase).state.max(2,:)   = [u0 umax uf];
limits(iphase).state.min(3,:)   = [v0 vmin vmin];
limits(iphase).state.max(3,:)   = [v0 vmax vmax];
limits(iphase).control.min(1,:) = u1min;
limits(iphase).control.max(1,:) = u1max;
limits(iphase).control.min(2,:) = u2min;
limits(iphase).control.max(2,:) = u2max;

limits(iphase).parameter.min    = [];
limits(iphase).parameter.max    = [];

limits(iphase).path.min    = 0;
limits(iphase).path.max    = 0;
limits(iphase).event.min   = 0;
limits(iphase).event.max   = 0;

guess(iphase).time =  [t0; tf];
guess(iphase).state(:,1) = [r0; rf];
guess(iphase).state(:,2) = [u0; uf];
guess(iphase).state(:,3) = [v0; vf];
guess(iphase).control(:,1) = [0; 0];
guess(iphase).control(:,2) = [1; 1];
guess(iphase).parameter = [];

%%
setup.limits = limits;
setup.guess = guess;
setup.linkages = [];

setup.name       = 'Maxradius';
setup.funcs.cost = 'MaxradiusCost';
setup.funcs.dae  = 'MaxradiusDae';
setup.funcs.event= 'maxradeventfun';

setup.solver = 'snopt';
setup.method = 'pseudospectral';
setup.checkDerivatives = 1;

%%

output = DMG(setup);

figure
plot(output.solution.time, output.solution.state(:,1))
hold on
plot(output.solution.time, output.solution.state(:,2))
plot(output.solution.time, output.solution.state(:,3))
hold off

figure(2)
plot(output.solution.time, atan2(output.solution.control(:,1),output.solution.control(:,2)))






