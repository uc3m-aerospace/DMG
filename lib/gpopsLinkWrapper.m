function link = gpopsLinkWrapper(t,xplink,extras);
%------------------------------------------------------------------%
% Wrapper function to evaluate linkage constraints                 %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

setup = extras.setup;
left_phase = extras.left.phase;
right_phase =  extras.right.phase;
nstates_left = extras.left.nstates;
nstates_right = extras.right.nstates;
nparameters_left = extras.left.nparameters;
nparameters_right = extras.right.nparameters;

xpleft = xplink(1:nstates_left+nparameters_left);
xpright = xplink(nstates_left+nparameters_left+1:end);
sol.left.phase = left_phase;
sol.left.state = xpleft(1:nstates_left);
sol.left.parameter = xpleft(nstates_left+1:nstates_left+nparameters_left);
sol.right.phase = right_phase;
sol.right.state = xpright(1:nstates_right);
sol.right.parameter = xpright(nstates_right+1:nstates_right+nparameters_right);
link = feval(setup.funcs.link,sol);
