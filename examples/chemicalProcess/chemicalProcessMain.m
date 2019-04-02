%-----------------------------------
% BEGIN: script chemicalProcessMain.m
%-----------------------------------
% This m-file is the main file for the following optimal control
% problem: 
%   minimize
%     J = int(x^4+0.5z^2+0.5u^2)dt
%   subject to
%     dx/dt = xz
%     dz/dt = 10(-z+u)
%     [x(0), z(0)]   = [1/sqrt(2) 0]
%     [x(tf), z(tf)] = [   0.5    0]
%------------------------------------------------------------------
% This example is taken from the following reference:
% Kokotovic, Petar; Khalil, Hassan; and O'Reilly, John, Singular
% Perturbation Methods in Control, Analysis and Design, SIAM, 1999.
% This problem was coded by Christopher L. Darby (cdarby@ufl.edu)
%------------------------------------------------------------------
clear setup limits guess

x0 = 1/sqrt(2);
xf = 0.5;
z0 = 0;
zf = 0;

xmin = -3;
xmax =  3;
zmin = -3;
zmax =  3;
umin = -3;
umax =  3;
t0min = 0;
t0max = 0;
tfmin = 1;
tfmax = 1;

% Phase 1 Information
iphase = 1;
limits(iphase).nodes = 50;
limits(iphase).time.min = [t0min tfmin];
limits(iphase).time.max = [t0max tfmax];
limits(iphase).state.min(1,:) = [x0 xmin xf];
limits(iphase).state.max(1,:) = [x0 xmax xf];
limits(iphase).state.min(2,:) = [z0 zmin zf];
limits(iphase).state.max(2,:) = [z0 zmax zf];
limits(iphase).control.min = umin;
limits(iphase).control.max = umax;
limits(iphase).parameter.min = [];
limits(iphase).parameter.max = [];
limits(iphase).path.min = [];
limits(iphase).path.max = [];
limits(iphase).event.min = [];
limits(iphase).event.max = [];
limits(iphase).duration.min = [];
limits(iphase).duration.max = [];
guess(iphase).time = [t0min; tfmax];
guess(iphase).state(:,1) = [x0; x0];
guess(iphase).state(:,2) = [z0; z0];
guess(iphase).control = [umax; umax];
guess(iphase).parameter = []; % No parameters in Phase 1

connections = [];
setup.name  = 'Chemical-Process-Problem';
setup.funcs.cost = 'chemicalProcessCost';
setup.funcs.dae = 'chemicalProcessDae';
setup.limits = limits;
setup.guess = guess;
setup.linkages = [];
setup.derivatives = 'analytic';
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='hermite-simpson';


output = DMG(setup);
solution = output.solution;





