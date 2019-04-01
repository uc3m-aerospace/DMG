%-------------------------------------
% BEGIN: function brachistochroneDae.m
%-------------------------------------
function dae = brachistochroneDae(sol,setup);

CONSTANTS = setup.CONSTANTS;

t = sol.time;
x = sol.state(:,1);
y = sol.state(:,2);
v = sol.state(:,3);
u = sol.control;
xdot = v.*sin(u);
ydot = -v.*cos(u);
vdot = CONSTANTS.g*cos(u);
dae = [xdot ydot vdot];
%-----------------------------------
% END: function brachistochroneDae.m
%-----------------------------------

