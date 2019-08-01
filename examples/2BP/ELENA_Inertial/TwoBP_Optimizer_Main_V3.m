% The system has been defined with 4 state variables: position (r),
% velocity(v), time(t), mass(m) with a cartesian system of reference.

%clc
%close all
clear setup limits guess;
format long
mission = load('mission_performance.txt');
%% Fixing the ctes of the problem
setup.mu_jupiter = 1.26686534e17;   % [m^3/s^2] Gravitational constant Jupiter
setup.go_earth = 9.80665 ;          % [m/s^2] Gravitational acceleration of Earth
setup.spec_impulse = 2500;          % [s] Specific impulse of the vehicle
setup.thrust = 0.05;                 % [N] Thrust of the vehicle
setup.r_callisto = 1.88270e9;        % [m] Orbital radius of callisto
setup.r_ganymede = 1.07040e9;        % [m] Orbital radius of ganymede
setup.initial_mass = 1000;           % [kg] Initial mass of the vehicle
setup.ratio_mass = 0.96;             % Ratio between final mass and initial mass
setup.time =231.84*24*3600;           % [s]Corresponds with the time for a Hohmann transfer

%% Fixing the caracteristic magnitudes
setup.factor = 1;
setup.lc = setup.r_ganymede*setup.factor;
setup.tc = sqrt(setup.lc^3/setup.mu_jupiter);
setup.vc = setup.lc/setup.tc;
setup.ac = setup.vc/setup.tc;
setup.mc = setup.initial_mass;

setup.mu = 1;
lc = setup.lc;
tc = setup.tc;
vc = setup.vc;
ac = setup.ac;
mc = setup.mc;

%% Fixing the non dimensional ctes
r_ganymede_ad = setup.r_ganymede/lc;
r_callisto_ad = setup.r_callisto/lc;
time_ad = setup.time/tc;
v_ganymede_ad = sqrt(setup.mu_jupiter/setup.r_ganymede)/vc;
v_callisto_ad = sqrt(setup.mu_jupiter/setup.r_callisto)/vc;

setup.r_ganymede_ad = r_ganymede_ad;
setup.r_callisto_ad = r_callisto_ad;
setup.v_ganymede_ad = v_ganymede_ad;
setup.v_callisto_ad = v_callisto_ad;
%% Initial conditions
t_0 = 0;
rx_0 =  mission(1,2)*1000/lc;
ry_0 =  mission(1,3)*1000/lc;
vx_0 =  mission(1,8)*1000/vc;
vy_0 =  mission(1,9)*1000/vc;
m_0 = 1;

%% Final conditions
t_f = time_ad;

rx_f = mission(end,2)*1000/setup.lc;
ry_f = mission(end,3)*1000/setup.lc;
vx_f = mission(end,8)*1000/setup.vc;
vy_f = mission(end,9)*1000/setup.vc;
m_f = m_0*setup.ratio_mass;
%% Minimum conditions
rx_min = -1.1*r_callisto_ad;
ry_min = -1.1*r_callisto_ad;
vx_min = -1.5*v_ganymede_ad;
vy_min = -1.5*v_ganymede_ad;
m_min = m_f;

%% Maximum conditions
rx_max = 2*r_callisto_ad;
ry_max = 2*r_callisto_ad;
vx_max = 2*v_ganymede_ad;
vy_max = 2*v_ganymede_ad;
m_max = m_0;
%



%% Control Variables
throttle_min = 1;
throttle_max = 1;
alpha_min    = -pi;
alpha_max    = pi;
setup.timef  = 100;

%% Setting the values for the phase
iphase = 1; % Setting the number of the phase
limits(iphase).nodes = 300; % Setting the number of nodes

limits(iphase).time.min = [t_0,(t_f-40*24*3600/setup.tc)/setup.timef];
limits(iphase).time.max = [t_0,(t_f+40*24*3600/setup.tc)/setup.timef];

limits(iphase).state.min(1,:)   = [rx_0 rx_min rx_f];
limits(iphase).state.max(1,:)   = [rx_0 rx_max rx_f];

limits(iphase).state.min(2,:)   = [ry_0 ry_min ry_f];
limits(iphase).state.max(2,:)   = [ry_0 ry_max ry_f];

limits(iphase).state.min(3,:)   = [vx_0 vx_min vx_f];
limits(iphase).state.max(3,:)   = [vx_0 vx_max vx_f];

limits(iphase).state.min(4,:)   = [vy_0 vy_min vy_f];
limits(iphase).state.max(4,:)   = [vy_0 vy_max vy_f];

limits(iphase).state.min(5,:)   = [m_0 0.1 0.1];
limits(iphase).state.max(5,:)   = [m_0 m_0 m_0];

limits(iphase).control.min(1,:) = throttle_min;
limits(iphase).control.max(1,:) = throttle_max;

limits(iphase).control.min(2,:) = alpha_min;
limits(iphase).control.max(2,:) = alpha_max;

limits(iphase).path.min    = [0.7];
limits(iphase).path.max    = [1.7];


%% Setting the guess for the phase
mission(:,6) = pi/180*mission(:,6);
angle_prime = zeros(length(mission(:,1)),1);
thrust_angle = zeros(length(mission(:,1)),1);
for i = 1:length(mission(:,1))
    angle_prime(i,1) = atan2(mission(i,3),mission(i,2));
    thrust_angle(i,1) = atan2((cos(mission(i,6))*cos(angle_prime(i,1)) + sin(mission(i,6))*sin(angle_prime(i,1))...
        ),(-cos(mission(i,6))*sin(angle_prime(i,1)) + sin(mission(i,6))*cos(angle_prime(i,1))));
end
%      
plot(mission(:,1)*24*3600/tc,thrust_angle(:,1))
guess(iphase).time         = mission(:,1)*24*3600/tc/setup.timef;
guess(iphase).state(:,1)   = mission(:,2)*1000/lc;
guess(iphase).state(:,2)   = mission(:,3)*1000/lc;
guess(iphase).state(:,3)   = mission(:,8)*1000/vc;
guess(iphase).state(:,4)   = mission(:,9)*1000/vc;
guess(iphase).state(:,5)   = linspace(1,m_f,length(mission(:,1)));
guess(iphase).control(:,1) = ones(size(mission(:,2)));
guess(iphase).control(:,2) = thrust_angle(:,1);

%% Setting the parameters
setup.limits = limits;
setup.guess = guess;
setup.linkages = [];

setup.name       = 'TwoBP_Optimizer_Results';
setup.funcs.cost = 'TwoBP_Optimizer_Cost';
setup.funcs.dae  = 'TwoBP_Optimizer_Dae';
%setup.funcs.event  = 'TwoBP_Optimizer_Event';
setup.autoscale  = 'off';
setup.derivatives = 'automatic';
setup.parallel    = 'no';
setup.ipopt.tol        = 1e-6;
setup.ipopt.bound_push = 1e-6;
setup.ipopt.mu_init    = 1e-2;

setup.solver ='ipopt';
setup.method ='hermite-simpson';

%% Running the solver
output = DMG(setup);

%% Drawing results
figure
plot(output.solution.state(:,1),output.solution.state(:,2),'g','LineWidth',2)
title('Non dimensional behaviour of the transfer orbit')
axis([-1.2*r_callisto_ad 1.2*r_callisto_ad -1.2*r_callisto_ad 1.2*r_callisto_ad])
legend('Optimal trajectory')
grid on
axis equal

figure ('Name','x coordinate vs time')
plot(output.solution.time,output.solution.state(:,1))
title('Non dimensional x coordinate vs time')

figure ('Name','y coordinate vs time')
plot(output.solution.time,output.solution.state(:,2))
title('Non dimensional y coordinate vs time')

figure ('Name','Mass vs time')
plot(output.solution.time,output.solution.state(:,5))
title('Non dimensional Mass vs time')

figure ('Name','Throttle vs time')
plot(output.solution.time,output.solution.control(:,1))
title('Trottle vs time')
axis([0 t_f 0 1.2])

figure ('Name','Thrust orientation vs time')
plot(output.solution.time,output.solution.control(:,2)*180/pi)
title('Trust orientation [º]  vs time')

disp(['The final mass ratio after the first iteration is ' num2str(output.solution.state(end,5))])