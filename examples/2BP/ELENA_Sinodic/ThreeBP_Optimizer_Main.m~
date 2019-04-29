% The system has been defined with 4 state variables: position (r),
% velocity(v), time(t), mass(m) with a cartesian system of reference.

%clc
close all
clear setup limits guess;
format long
global constants
mission = load('mission_performance.txt');

%% Fixing the ctes of the problem
    constants.mu_jupiter = 1.26686534e17;   % [m^3/s^2] Gravitational constant Jupiter
    constants.go_earth = 9.80665 ;          % [m/s^2] Gravitational acceleration of Earth
    constants.spec_impulse = 2500;          % [s] Specific impulse of the vehicle
    constants.thrust = 0.05;                % [N] Thrust of the vehicle
    constants.r_callisto = 1.8827e9;        % [m] Orbital radius of callisto
    constants.r_ganymede = 1.0704e9;        % [m] Orbital radius of ganymede
    constants.initial_mass = 1000;          % [kg] Initial mass of the vehicle
    constants.ratio_mass = 0.96;            % Ratio between final mass and initial mass
    constants.time = 231.84*24*3600;         % [s]Corresponds with the time for a Hohmann transfer
    
%% Fixing the caracteristic magnitudes
    constants.lc = constants.r_ganymede;
    constants.tc = sqrt(constants.lc^3/constants.mu_jupiter);
    constants.vc = constants.lc/constants.tc;
    constants.ac = constants.vc/constants.tc;
    constants.mc = constants.initial_mass;
    
    lc = constants.lc;
    tc = constants.tc;
    vc = constants.vc;
    ac = constants.ac;
    mc = constants.mc;
    constants.w = 1; 
    w = constants.w;
%% Fixing the non dimensional ctes
    r_ganymede_ad = constants.r_ganymede/lc;
    r_callisto_ad = constants.r_callisto/lc;
    time_ad       = constants.time/tc;
    v_ganymede_ad = sqrt(constants.mu_jupiter/constants.r_ganymede)/vc;
    v_callisto_ad = sqrt(constants.mu_jupiter/constants.r_callisto)/vc;
    
%% Changing the reference system to the synodic
    %Position and velocity vector
    r(1,:) = mission(:,2)*1000/lc;
    r(2,:) = mission(:,3)*1000/lc;
    r(3,:) = 0;
    v(1,:) = mission(:,8)*1000/vc;
    v(2,:) = mission(:,9)*1000/vc;
    v(3,:) = 0;
    t(1,:) = mission(:,1)*24*3600/tc;
    omega = [0; 0; w];
    r_prime = zeros(3,length(mission(:,1)));
    v_prime = zeros(3,length(mission(:,1)));
    rot_matrix = zeros(3,3,length(mission(:,1)));
    for i = 1:numel(r(1,:))
        rot_matrix   = [cos(w*t(i)) sin(w*t(i)) 0; -sin(w*t(i)) cos(w*t(i)) 0; 0 0 1];
        r_prime(:,i) = rot_matrix*r(:,i);
        v_prime(:,i) = rot_matrix*(v(:,i) - cross(+omega,r(:,i)));
    end
    
    %Trust angle 
    angle_prime = zeros(length(mission(:,1)),1);
    T = zeros(2,length(mission(:,1)));
    T_prime = zeros(2,length(mission(:,1)));
    thrust_angle = zeros(length(mission(:,1)),1);
    for i = 1:length(mission(:,1))
        angle_prime(i,1) = atan2(mission(i,3),mission(i,2));
        T(1,i) = -cos(mission(i,6)*pi/180)*sin(angle_prime(i,1)) + sin(mission(i,6)*pi/180)*cos(angle_prime(i,1));
        T(2,i) =  cos(mission(i,6)*pi/180)*cos(angle_prime(i,1)) + sin(mission(i,6)*pi/180)*sin(angle_prime(i,1));
        rot_matrix = [cos(w*t(i)) sin(w*t(i)); -sin(w*t(i)) cos(w*t(i))];
        T_prime(:,i) = rot_matrix*T(:,i);
        thrust_angle(i,1) = atan2(T_prime(2,i),T_prime(1,i));
    end
    
%% Initial conditions
    t_0 = 0;
    rx_0 = r_prime(1,1);
    ry_0 = r_prime(2,1);
    vx_0 = v_prime(1,1);
    vy_0 = v_prime(2,1);
    m_0 = 1;

%% Final conditions 
    t_f  = time_ad;
    rx_f = r_prime(1,end);
    ry_f = r_prime(2,end);
    vx_f = v_prime(1,end);
    vy_f = v_prime(2,end);
    m_f = m_0*constants.ratio_mass;

%% Minimum conditions
    rx_min = 1.5*min(r_prime(1,:));
    ry_min = 1.5*min(r_prime(2,:));
    vx_min = 1.5*min(v_prime(1,:));
    vy_min = 1.5*min(v_prime(2,:));
    m_min = m_f;

%% Maximum conditions
    rx_max = 1.5*max(r_prime(1,:));
    ry_max = 1.5*max(r_prime(2,:));
    vx_max = 1.5*max(v_prime(1,:));
    vy_max = 1.5*max(v_prime(2,:));
    m_max = m_0;

%% Control Variables
    throttle_min = 1;
    throttle_max = 1;
    alpha_min = -pi;
    alpha_max = pi;
    
%% Setting the values for the phase  
    setup.timef  = 100;
    iphase = 1; % Setting the number of the phase
    limits(iphase).nodes = 500; % Setting the number of nodes

    limits(iphase).time.min = [t_0,(t_f-40*24*3600/tc)/setup.timef];
    limits(iphase).time.max = [t_0,(t_f+40*24*3600/tc)/setup.timef];

    limits(iphase).state.min(1,:)   = [rx_0 rx_min rx_f];
    limits(iphase).state.max(1,:)   = [rx_0 rx_max rx_f];

    limits(iphase).state.min(2,:)   = [ry_0 ry_min ry_f];
    limits(iphase).state.max(2,:)   = [ry_0 ry_max ry_f];

    limits(iphase).state.min(3,:)   = [vx_0 vx_min vx_f];
    limits(iphase).state.max(3,:)   = [vx_0 vx_max vx_f];

    limits(iphase).state.min(4,:)   = [vy_0 vy_min vy_f];
    limits(iphase).state.max(4,:)   = [vy_0 vy_max vy_f];

    limits(iphase).state.min(5,:)   = [m_0 0 0];
    limits(iphase).state.max(5,:)   = [m_0 m_0 m_0];
       
    limits(iphase).control.min(1,:) = throttle_min;
    limits(iphase).control.max(1,:) = throttle_max;
    
    limits(iphase).control.min(2,:) = alpha_min;
    limits(iphase).control.max(2,:) = alpha_max;
   
    limits(iphase).path.min    = 0.7;
    limits(iphase).path.max    = 1.8;
    
%% Setting the guess for the phase    
    guess(iphase).time       =  transpose(t(1,:))/setup.timef;
    guess(iphase).state(:,1) = transpose(r_prime(1,:));
    guess(iphase).state(:,2) = transpose(r_prime(2,:));
    guess(iphase).state(:,3) = transpose(v_prime(1,:));
    guess(iphase).state(:,4) = transpose(v_prime(2,:));
    guess(iphase).state(:,5) = linspace(1,m_f,length(mission(:,1)));
    guess(iphase).control(:,1) = ones(size(thrust_angle(:,1)));
    guess(iphase).control(:,2) = thrust_angle(:,1);
    guess(iphase).parameter = []; 

%% Setting the parameters
    setup.limits = limits;
    setup.guess = guess;
    setup.linkages = [];

    setup.name       = 'ThreeBP_Optimizer_Results';
    setup.funcs.cost = 'ThreeBP_Optimizer_Cost';
    setup.funcs.dae  = 'ThreeBP_Optimizer_Dae';
    setup.autoscale  = 'off';
    setup.derivatives = 'automatic';
    setup.parallel    = 'no';
    setup.ipopt.tol        = 1e-6;
    setup.ipopt.bound_push = 1e-6;
    setup.ipopt.mu_init    = 1e-2;
    setup.solver ='ipopt';
    setup.method ='hermite-simpson';
    setup.checkDerivatives = 1;

%% Running the solver
    output = DMG(setup);
    
% %% Drawing results
%     plot(output.solution.state(:,1),output.solution.state(:,2),'g','LineWidth',2)
%     title('Non dimensional behaviour of the transfer orbit')
%     axis([-1.2*r_callisto_ad 1.2*r_callisto_ad -1.2*r_callisto_ad 1.2*r_callisto_ad])
%     legend('Optimal trajectory')
%     
%     figure ('Name','x coordinate vs time')
%     plot(output.solution.time,output.solution.state(:,1))
%     title('Non dimensional x coordinate vs time')
%     
%     figure ('Name','y coordinate vs time')
%     plot(output.solution.time,output.solution.state(:,2))
%     title('Non dimensional y coordinate vs time')
%     
%     figure ('Name','Mass vs time')
%     plot(output.solution.time,output.solution.state(:,5))
%     title('Non dimensional Mass vs time')
% 
%     figure ('Name','Throttle vs time')
%     plot(output.solution.time,output.solution.control(:,1))
%     title('Trottle vs time')
%     axis([0 t_f 0 1.2])
%     
%     figure ('Name','Thrust orientation vs time')
%     plot(output.solution.time,output.solution.control(:,2)*180/pi)
%     title('Trust orientation [º]  vs time')
%     
%     disp(['The final mass ratio after the first iteration is ' num2str(output.solution.state(end,5))])