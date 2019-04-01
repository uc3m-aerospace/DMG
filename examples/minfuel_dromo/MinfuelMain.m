% ----------------------------
% Bryson Maximum-Radius Example
% ----------------------------
% This problem is taken verbatim from the following reference:
% Bryson, A. E. and Ho, Y-C, Applied Optimal Control, Hemisphere
% Publishing, New York, 1975.
% ----------------------------
clear setup limits guess;
format long

global constants
constants.p1 = 1/0.1405;
%maxrad.spc = which('maxrad.spc');
%snspec (maxrad.spc);
snseti ('Major Iteration limit', 300);
snseti ('Minor Iteration limit',300);
snseti ('Minor feasibility tolerance', 1.0e-10);
snseti ('Major feasibility tolerance',1.0e-10);
snseti ('Major optimality tolerance',1.0e-10);
snseti ('Scale option',1);

rf=1.525243822099782;
C10 = 0;
C20 = 0;
C30 = 1;
tau0 = 0;
C1f= 0;
C2f= 0 ;
C3f= sqrt(1/rf);
tauf=3.32;
C1min = -1;
C1max =  1;
C2min= -1;
C2max = 1;
C3min = -1;
C3max = 1;
taumin = 0;
taumax = 3.32;
u1min = -5;
u1max = -u1min;
u2min = u1min;
u2max = u1max;
sigma0 = 0 ;
sigmaf = 1;

iphase = 1;
limits(iphase).nodes = 30;
limits(iphase).time.min         = [0     sigmaf];
limits(iphase).time.max         = [0     sigmaf];
limits(iphase).state.min(1,:)   = [C10 C1min C1f];
limits(iphase).state.max(1,:)   = [C10 C1max C1f];
limits(iphase).state.min(2,:)   = [C20 C2min C2f];
limits(iphase).state.max(2,:)   = [C20 C2max C2f];
limits(iphase).state.min(3,:)   = [C30 C3min C3f];
limits(iphase).state.max(3,:)   = [C30 C3max C3f];
limits(iphase).state.min(4,:)   = [tau0 taumin tauf];
limits(iphase).state.max(4,:)   = [tau0 taumax tauf];
limits(iphase).control.min(1,:) = u1min;
limits(iphase).control.max(1,:) = u1max;
limits(iphase).control.min(2,:) = u2min;
limits(iphase).control.max(2,:) = u2max;

limits(iphase).parameter.min    = [0 0 ]';
limits(iphase).parameter.max    = [2 5 ]';

limits(iphase).path.min    = 1;
limits(iphase).path.max    = 1;
limits(iphase).event.min   = 0;
limits(iphase).event.max   = 0;
guess(iphase).time =  [0; sigmaf];
guess(iphase).state(:,1) = [C10; C1f];
guess(iphase).state(:,2) = [C20; C2f];
guess(iphase).state(:,3) = [C30; C3f];
guess(iphase).state(:,4) = [tau0; tauf];
guess(iphase).control(:,2) = [1; 0];
guess(iphase).parameter =  [0.5 2]' ;

setup.name = 'Minfuel';
setup.funcs.cost = 'MinfuelCost';
setup.funcs.dae = 'MinfuelDae';
setup.funcs.event = 'minfueleventfun';
setup.limits = limits;
setup.derivatives = 'automatic';
setup.checkDerivatives = 0;
setup.guess = guess;
setup.linkages = [];
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='pseudospectral';

output = DMG(setup);
solution = output.solution;
figure(1)
plot(solution.state(:,4),solution.state(:,1))
figure(2)
plot(solution.state(:,4),solution.state(:,2))
figure(3)
plot(solution.state(:,4),solution.state(:,3))
figure(4)
plot(solution.state(:,4), atan2(solution.control(:,1),solution.control(:,2)))
figure(5)
plot(solution.state(:,4), solution.costate)
solution.costate(1,:)

