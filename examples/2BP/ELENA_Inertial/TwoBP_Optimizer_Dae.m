function [dae,Ddae] = TwoBP_Optimizer_Dae(sol,setup)

%% State variables definition
    t = sol.time;   % Definition of the time array
    x = sol.state;  % Definition of the state array
    rx = x(:,1);    % Definition of the distance vector in the x axis 
    ry = x(:,2);    % Definition of the distance vector in the y axis
    vx = x(:,3);    % Definition of the velocity vector in the x axis
    vy = x(:,4);    % Definition of the velocity vector in the y axis
    m  = x(:,5);     % Definition of the mass vector

%% Control variables definition
    con      = sol.control;      % Definition of the state array
    throttle = con(:,1);    % Definition of the throttle vector
    alpha    = con(:,2);       % Definition of the throttle vector

%% setup definition
    mu_jupiter = setup.mu;                                 % Non dimensional Gravitational constant Jupiter
    go_earth = setup.go_earth/setup.ac;             % Non dimensional Gravitational acceleration of Earth
    spec_impulse = setup.spec_impulse/setup.tc;     % Non dimensional Specific impulse of the
    thrust = setup.thrust/(setup.mc*setup.ac);      % Non dimensional Thrust of the vehicle
    % Equations setting
    r = sqrt(rx.^2+ry.^2);

    rxdot = vx ;
    rydot = vy;
    vxdot = -mu_jupiter./(r.^3).*rx + thrust.*cos(alpha).*throttle./m ;
    vydot = -mu_jupiter./(r.^3).*ry + thrust.*sin(alpha).*throttle./m ;
    mdot =  -thrust.*throttle./(go_earth*spec_impulse);
    
    %
    path = sqrt(rx.^2+ry.^2);

    
    dae = [rxdot*setup.timef rydot*setup.timef vxdot*setup.timef vydot*setup.timef mdot*setup.timef path];
    %
