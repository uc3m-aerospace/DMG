function event = gpopsEventWrapper(t,xup,extras);
%------------------------------------------------------------------%
% Wrapper function to evaluate event constraints                   %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

setup = extras.setup;
iphase = extras.phase;
nstates = extras.nstates;
ncontrols = extras.ncontrols;
nparameters = extras.nparameters;
init = xup(1:nstates+1);
term = xup(nstates+2:2*(nstates+1));
p = xup(2*(nstates+1)+1:end);
sol.initial.time = init(1);
sol.initial.state = init(2:nstates+1);
sol.terminal.time = term(1);
sol.terminal.state = term(2:nstates+1);
sol.parameter = p;
sol.phase = iphase;
event = feval(setup.funcs.event,sol,setup);
