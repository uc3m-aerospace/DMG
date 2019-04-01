function [event Devent] = launchEvent(sol,setup);

CONSTANTS = setup.CONSTANTS;
t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
p  = sol.parameter;
iphase = sol.phase;

if iphase==4,
    oe = launchrv2oe(xf(1:3),xf(4:6),CONSTANTS.mu);
    event = oe(1:5);
else
    event = [];
end;

% avoid calc of derivs in not necessary
if nargout == 2

    if iphase == 4
        Doe = launchrv2oe_D(xf(1:3),xf(4:6),CONSTANTS.mu,CONSTANTS.Re);

        % Devents = [dE/dx0, dE/dt0, dE/dxf, dE/dtf, dE/dp]
        lx0 = length(x0);
        lp  = length(p);
        Devent = [zeros(5,lx0), zeros(5,1), [Doe, zeros(5,1)], zeros(5,1), zeros(5,lp)]; 
    else
        Devent = [];
    end
end

