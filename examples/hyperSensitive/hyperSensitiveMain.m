%-----------------------------------
% BEGIN: script hyperSensitiveMain.m
%-----------------------------------
% This m-file is the main file for the following optimal control
% problem: 
%   minimize
%     J = 0.5*(x^2+u^2)
%   subject to
%     dx/dt = -x^3 + u
%     x(0) = 1.5
%     x(tf) = 1
%------------------------------------------------------------------
% This example is taken from the following reference:
% Rao, A. V. and Mease, K. D., "Eigenvector Approximate Dichotomic
% Basis Method for Solving Hypersensitive Optimal Control
% Problems," Optimal Control Applications and Methods, Vol. 21,
% No. 1, 2000, pp. 1-19.
%------------------------------------------------------------------

clear setup limits guess

x0 = 1.5;
xf = 1;
xmin = -50;
xmax =  50;
umin = -50;
umax =  50;

iphase = 1;
limits(iphase).nodes = 50;
limits(iphase).time.min = [0 50];
limits(iphase).time.max = [0 50];
limits(iphase).state.min = [x0 xmin xf];
limits(iphase).state.max = [x0 xmax xf];
limits(iphase).control.min = umin;
limits(iphase).control.max = umax;
limits(iphase).parameter.min = [];
limits(iphase).parameter.max = [];
limits(iphase).path.min = [];
limits(iphase).path.max = [];
limits(iphase).event.min = [];
limits(iphase).event.max = [];
guess(iphase).time = [0; 20];
guess(iphase).state(:,1) = [x0; x0];
guess(iphase).control = [0; 0];
guess(iphase).parameter = [];

clear x0 xf xmin xmax umin umax

setup.name  = 'HyperSensitive-Problem';
setup.funcs.cost = 'hyperSensitiveCost';
setup.funcs.dae = 'hyperSensitiveDae';
setup.linkages = [];
setup.limits = limits;
setup.guess = guess;
setup.derivatives = 'automatic';
setup.checkDerivatives = 0;
setup.direction = 'increasing';
setup.autoscale = 'off';

output = gpops(setup);

%-----------------------------------
% END: script hyperSensitiveMain.m
%-----------------------------------
