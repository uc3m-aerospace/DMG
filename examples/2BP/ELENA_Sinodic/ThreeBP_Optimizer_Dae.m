function [dae,Ddae] = TwoBP_Optimizer_Dae(sol,setup)

global constants
%% State variables definition
    t = sol.time;   % Definition of the time array
    x = sol.state;  % Definition of the state array
    rx = x(:,1);    % Definition of the distance vector in the x axis 
    ry = x(:,2);    % Definition of the distance vector in the y axis
    vx = x(:,3);    % Definition of the velocity vector in the x axis
    vy = x(:,4);    % Definition of the velocity vector in the y axis
    m = x(:,5);     % Definition of the mass vector
    
%% Control variables definition
    con = sol.control;      % Definition of the state array
    throttle = con(:,1);    % Definition of the throttle vector
    alpha = con(:,2);       % Definition of the throttle vector

%% Constants definition
    mu_jupiter  = 1;                                         % Non dimensional Gravitational constant Jupiter
    mu_ganymede = 0;
%     omega = constants.w;
    go_earth = constants.go_earth/constants.ac;             % Non dimensional Gravitational acceleration of Earth
    spec_impulse = constants.spec_impulse/constants.tc;     % Non dimensional Specific impulse of the
    thrust = constants.thrust/(constants.mc*constants.ac);  % Non dimensional Thrust of the vehicle

%% Equations setting
    rxdot = vx;
    rydot = vy;
    vxdot = (-mu_jupiter./((rx.^2+ry.^2).^(3/2))).*rx +(-mu_ganymede./(((-1+rx).^2+ry.^2).^(3/2))).*(-1+rx) + thrust.*cos(alpha).*throttle./m + rx + 2*vy;
    vydot = (-mu_jupiter./((rx.^2+ry.^2).^(3/2))).*ry +(-mu_ganymede./(((-1+rx).^2+ry.^2).^(3/2))).*(ry)    + thrust.*sin(alpha).*throttle./m + ry - 2*vx;
    mdot = -thrust.*throttle./(go_earth*spec_impulse);
    path = sqrt(rx.^2+ry.^2);
    dae = [rxdot*setup.timef rydot*setup.timef vxdot*setup.timef vydot*setup.timef mdot*setup.timef path];

