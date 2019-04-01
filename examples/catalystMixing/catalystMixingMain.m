%-----------------------------------
% BEGIN: script catalystMixingMain.m
%-----------------------------------
% Maximize the objective functional
%
%    J = 1-x1(tf)-x2(tf)
%
% subject to the dynamic constraints
%
%    x1dot = u*(10*x2-x1)
%    x2dot = u*(x1-10*x2)-(1-u)*x2
%
% with the path (control) constraint
%    0 <= u <= 1
% and the initial conditions
%    x1(0) = 1
%    x2(0) = 0
%-----------------------------------
clear setup limits guess

t0         = 0;
tfMax      = 1;
tfGuess    = 1;

iphase = 1;
limits(iphase).nodes           = 100;
limits(iphase).time.min        = [0 tfMax];
limits(iphase).time.max        = [0 tfMax];
limits(iphase).state.min(1,:)  = [1 0.9 0.80];
limits(iphase).state.max(1,:)  = [1 1 0.95];
limits(iphase).state.min(2,:)  = [0 0 0];
limits(iphase).state.max(2,:)  = [0 0.1 0.1];
limits(iphase).control.min     = 0;
limits(iphase).control.max     = 1;
limits(iphase).parameter.min   = [];
limits(iphase).parameter.max   = [];
limits(iphase).duration.min    = [];
limits(iphase).duration.max    = [];
guess(1).time                  = [t0; tfGuess];
guess(1).state(:,1)            = [1; 1];
guess(1).state(:,2)            = [0; 0];
guess(1).control(:,1)          = [1; 1];
guess(iphase).parameter        = [];

linkages = [];

setup.name        = 'Catalyst-Mixing-Problem';
setup.funcs.cost  = 'catalystMixingCost';
setup.funcs.dae   = 'catalystMixingDae';
setup.derivatives = 'automatic';
setup.autoscale   = 'off';
setup.direction   = 'increasing';
setup.limits      = limits;
setup.linkages    = linkages;
setup.guess       = guess;
setup.solver ='ipopt';
setup.method ='pseudospectral';


output            = DMG(setup);
solution          = output.solution;



