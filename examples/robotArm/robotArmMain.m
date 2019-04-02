% -----------------
% Robot Arm Example
% -----------------
% This example is taken verbatim from the following reference:
%   Benchmarking Optimization Software with COPS Elizabeth D. Dolan
%   and Jorge J. More ARGONNE NATIONAL LABORATORY

clear setup limits guess
global CONSTANTS;

L = 5;
CONSTANTS.L = L;

t0 = 0;
x10 = 4.5;
x1f = 4.5;
x1min = 0;
x1max = L;
x20 = 0;
x2f = 0;
x2min = -10*L;
x2max =  10*L;
x30 = 0;
x3f = 2*pi/3;
x3min = -pi;
x3max =  pi;
x40 = 0;
x4f = 0;
x4min = -50;
x4max =  50;
x50 = pi/4;
x5f = pi/4;
x5min =  0;
x5max =  pi;
x60 = 0;
x6f = 0;
x6min = -50;
x6max =  50;
u1min = -1;
u1max =  1;
u2min = -1;
u2max =  1;
u3min = -1;
u3max =  1;
t0 = 0;
tfMin = 0.1;
tfMax= 10;

iphase = 1;
limits(iphase).nodes = 60;
limits(iphase).time.min    = [t0 tfMin];
limits(iphase).time.max    = [t0 tfMax];
limits(iphase).state.min(1,:) = [x10 x1min x1f];
limits(iphase).state.max(1,:) = [x10 x1max x1f];
limits(iphase).state.min(2,:) = [x20 x2min x2f];
limits(iphase).state.max(2,:) = [x20 x2max x2f];
limits(iphase).state.min(3,:) = [x30 x3min x3f];
limits(iphase).state.max(3,:) = [x30 x3max x3f];
limits(iphase).state.min(4,:) = [x40 x4min x4f];
limits(iphase).state.max(4,:) = [x40 x4max x4f];
limits(iphase).state.min(5,:) = [x50 x5min x5f];
limits(iphase).state.max(5,:) = [x50 x5max x5f];
limits(iphase).state.min(6,:) = [x60 x6min x6f];
limits(iphase).state.max(6,:) = [x60 x6max x6f];
limits(iphase).control.min(1,:) = u1min;
limits(iphase).control.max(1,:) = u1max;
limits(iphase).control.min(2,:) = u2min;
limits(iphase).control.max(2,:) = u2max;
limits(iphase).control.min(3,:) = u3min;
limits(iphase).control.max(3,:) = u3max;
limits(iphase).parameter.min = [];
limits(iphase).parameter.max = [];
limits(iphase).path.min      = [];
limits(iphase).path.max      = [];
limits(iphase).event.min     = [];
limits(iphase).event.max     = [];
guess(iphase).time            = [0; 1];
guess(iphase).state(:,1)     = [x10; x10];
guess(iphase).state(:,2)     = [x20; x20];
guess(iphase).state(:,3)     = [x30; x30];
guess(iphase).state(:,4)     = [x40; x40];
guess(iphase).state(:,5)     = [x50; x50];
guess(iphase).state(:,6)     = [x60; x60];
guess(iphase).control(:,1)   = [0; 0];
guess(iphase).control(:,2)   = [0; 0];
guess(iphase).control(:,3)   = [0; 0];
guess(iphase).parameter      = [];

setup.CONSTANTS = CONSTANTS;
setup.name = 'Robot-Arm-Rotation';
setup.limits = limits;
setup.guess = guess;
setup.funcs.cost = 'robotArmCost';
setup.funcs.dae  = 'robotArmDae';
setup.linkages = [];
setup.derivatives = 'automatic';
setup.autoscale = 'off';
setup.parallel  = 'no';
setup.solver ='ipopt';
setup.method ='hermite-simpson';
output = DMG(setup);
solution = output.solution;
