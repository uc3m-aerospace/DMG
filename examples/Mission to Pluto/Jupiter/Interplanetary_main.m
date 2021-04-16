% ----------------------------------------------
% Interplanetary Trajectories Problem
% ----------------------------------------------
clear setup limits guess linkages
%
addpath('/Users/davidmorante/Desktop/MicePackage')
EM  = load_spice_kernels('/Users/davidmorante/Desktop/MicePackage/');
%
% ----------------------------------------------
% MISSION SETUP
% ----------------------------------------------
setup.departure_planet = {'399'};
setup.arrival_planet   = {'9'};
setup.launcher         = 'Atlas_V_551_STAR48';
vinf_min               = 3;
vinf_max               = 10;
setup.flyby_planet     = [{'599'}];
setup.Initial_Date     = '2025 Jan 01 00:00:00';
setup.Final_Date       = '2035 Dec 31 00:00:00';
initial_date           = 0.3594;
vinf0_guess            = sqrt(76.88);
setup.tfMax            = 24;               % years
setup.hpmin            = [0,-0];         % m
setup.hpmax            = [3000000000,3000000000]; % km
Isp                    = [1000, 3000];
Ispguess               = 1000;
hpguess = 300;
cguess  = 0;
%
% ----------------------------------------------
% SPACECRAFT PARAMETERS
% ----------------------------------------------
setup.m0        = 1800; %Kg
setup.P0        = 1;   %KW
setup.g0        = 9.806650; %m/s
setup.Isp       = 3000;
%-----------------------------------------------
setup.lc  = 149597870.700e03;
setup.mu  = 132712440018e09;
setup.tc  = sqrt(setup.lc^3/setup.mu);
setup.vc  = setup.lc/setup.tc;
setup.ac  = setup.lc/setup.tc^2;
%
% Thrust for initial guess
%
setup.Tmax = 0.18/setup.ac/setup.m0;%
%
%
% Get Initial Conditions
%
setup.et0 = cspice_str2et(setup.Initial_Date);
t0        = 0;
tfMin     = 0;
%
% Final Time
%
tfMax1 = setup.tfMax*365*24*3600/setup.tc;
%
% Limits
%
rmin  = 0.9;
rmax  = 60;
xmin  = -60;
xmax  = 60;
ymin  = -60;
ymax  = 60;
zmin  = -20;
zmax  = 20;
vxmin = -7;
vxmax = 7;
vymin = -7;
vymax = 7;
vzmin = -7;
vzmax = 7;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ALL PHASES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
limits = struct;
guess  = struct;
%
m_0 = 1;
%
n_phases = numel(setup.flyby_planet) + 1;
%
setup.n_phases = n_phases;
%
tprev = 0;
%
for iphase = 1:n_phases
    %
    % STATE VARIABLE
    %
    limits(iphase).nodes  = 90;
    if iphase >1
        limits(iphase).nodes = 50;
    end
    %
    limits(iphase).time.min = [t0 tfMin];
    %
    if iphase == 1
        limits(iphase).time.max = [t0 tfMax1];
    else
        limits(iphase).time.max = [tfMax1 tfMax1];
    end
    %
    if iphase == n_phases
        limits(iphase).time.min = [t0 tfMin];
    end
    %
    limits(iphase).state.min(1,:)   = [xmin xmin xmin];
    limits(iphase).state.max(1,:)   = [xmax xmax xmax];
    limits(iphase).state.min(2,:)   = [ymin ymin ymin];
    limits(iphase).state.max(2,:)   = [ymax ymax ymax];
    limits(iphase).state.min(3,:)   = [zmin zmin zmin];
    limits(iphase).state.max(3,:)   = [zmax zmax zmax];
    limits(iphase).state.min(4,:)   = [vxmin vxmin vxmin];
    limits(iphase).state.max(4,:)   = [vxmax vxmax vxmax];
    limits(iphase).state.min(5,:)   = [vymin vymin vymin];
    limits(iphase).state.max(5,:)   = [vymax vymax vymax];
    limits(iphase).state.min(6,:)   = [vzmin vzmin vzmin];
    limits(iphase).state.max(6,:)   = [vzmax vzmax vzmax];
    if iphase == 1
        limits(iphase).state.min(7,:)   = [1 1e-2 1e-2];
    else
        limits(iphase).state.min(7,:)   = [1e-2 1e-2 1e-2];
    end
    limits(iphase).state.max(7,:)   = [1 1 1];
    % CONTROL
    limits(iphase).control.min(1,:) = 0;
    limits(iphase).control.max(1,:) = 1;
    %
    if iphase == 1

    end
    %
    limits(iphase).control.min(2,:) = -pi;
    limits(iphase).control.max(2,:) = pi; % alpha (inplane-angle)
    limits(iphase).control.min(3,:) = -pi;
    limits(iphase).control.max(3,:) =  pi; % beta (outplane-angle)
    limits(iphase).control.min(4,:) = Isp(1)/1000;
    limits(iphase).control.max(4,:) = Isp(2)/1000; % Isp (sec)
    %
    % PAREMETERS
    %
    if iphase == 1
        limits(iphase).parameter.min    = [setup.hpmin(iphase)/1e6;-pi;-pi;-pi;0;vinf_min ;0];
        limits(iphase).parameter.max    = [setup.hpmax(iphase)/1e6;pi; pi; pi;1;vinf_max  ;1];
    elseif iphase < n_phases
        limits(iphase).parameter.min    = [setup.hpmin(iphase)/1e6;-pi;0;vinf_min;0];
        limits(iphase).parameter.max    = [setup.hpmax(iphase)/1e6;pi;1;vinf_max; 1];
    else
        limits(iphase).parameter.min    = [0;vinf_min ;0];
        limits(iphase).parameter.max    = [1;vinf_max ;1];
    end
    %
    % CONSTRAINTS LIMITS
    %
    limits(iphase).path.min    = [];
    limits(iphase).path.max    = [];
    %
    if iphase == 1
        limits(iphase).event.min   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ;0*24*3600/setup.tc];
        limits(iphase).event.max   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ;Inf];
    else
        limits(iphase).event.min   = [ 0 ; 0 ; 0 ;0*24*3600/setup.tc];
        limits(iphase).event.max   = [ 0 ; 0 ; 0 ;Inf];
    end
    %
    limits(iphase).path.min = [rmin;0];
    limits(iphase).path.max = [rmax;0];
    %
    if iphase == setup.n_phases
        limits(iphase).event.min   = [limits(iphase).event.min ;  0 ;0;0];
        limits(iphase).event.max   = [limits(iphase).event.max ; 0;0;0];
    end
    %
    % --------------------------------------------
    % INITIAL GUESS PHASE 1
    % --------------------------------------------
    %
    initial_guess_1 = load(['Trajectory_Leg',num2str(iphase),'.output']);
    %
    t_guess     = initial_guess_1(:,1) + tprev;
    r_guess     = initial_guess_1(:,2);
    v_guess     = initial_guess_1(:,3);
    psi_guess   = initial_guess_1(:,4);
    T_guess     = initial_guess_1(:,5);
    alpha_guess = initial_guess_1(:,6);
    theta_guess = initial_guess_1(:,7);
    %m_guess     = linspace(1,0.9,numel(t_guess));
    %
    ind         = [true;not(t_guess(2:end) == t_guess(1:end-1))];
    %
    %t_guess    = t_guess*2;
    t_guess     = t_guess(ind) ;
    r_guess     = r_guess(ind);
    v_guess     = v_guess(ind);
    psi_guess   = psi_guess(ind);
    T_guess     = T_guess(ind);
    alpha_guess = wrapToPi(alpha_guess(ind));
    theta_guess = theta_guess(ind);
    m_guess     = m_0*exp(-mean(T_guess(2:end)*0.1)/(setup.g0/setup.ac*setup.Isp/setup.tc).*(t_guess-t_guess(1)));
    % m_guess     = m_guess(ind);
    %
    tprev       = t_guess(end);
    %
    m_0 = m_guess(end);
    %
    x_guess  = r_guess.*   cos(theta_guess);
    y_guess  = r_guess.*   sin(theta_guess);
    vx_guess = v_guess.* ( cos(psi_guess).*cos(theta_guess) - sin(psi_guess).*sin(theta_guess) );
    vy_guess = v_guess.* ( cos(psi_guess).*sin(theta_guess) + sin(psi_guess).*cos(theta_guess) );
    %
    guess(iphase).time         = t_guess;
    guess(iphase).state(:,1)   = x_guess;
    guess(iphase).state(:,2)   = y_guess;
    guess(iphase).state(:,3)   = 0;
    guess(iphase).state(:,4)   = vx_guess;
    guess(iphase).state(:,5)   = vy_guess;
    guess(iphase).state(:,6)   = 0;
    guess(iphase).state(:,7)   = m_guess;
    guess(iphase).control(:,1) = ones(size(T_guess)).*(T_guess>0);
    guess(iphase).control(:,2) = alpha_guess;
    guess(iphase).control(:,3) = 0;
    guess(iphase).control(:,4) = Ispguess/1000;
    %
    %plot(t_guess,m_guess)
    %hold on
    if iphase == 1
        %
        guess(iphase).parameter    = [hpguess/1e6;cguess;0;0;1;vinf0_guess;initial_date];
        %
    elseif iphase < n_phases
        %
        guess(iphase).parameter    = [hpguess/1e6;cguess;1;vinf0_guess;initial_date];
        %
    else
        %
        guess(iphase).parameter    = [1;vinf0_guess;initial_date];
        %
    end
    %
    % ----------------------------------------------------------------------------------------
end

%guess = solution;
%
%
%guess(1).time = solution(1).time;
%guess(1).state = solution(1).state;
%guess(1).control = solution(1).control;
%guess(1).parameter = solution(1).parameter;
%
%
% --------------------------------------------
% LINKAGES CONSTRAINTS
% --------------------------------------------
%
%
for ipair = 1:n_phases-1
    %
    ipair = ipair;
    linkages(ipair).left.phase  = ipair;
    linkages(ipair).right.phase = ipair + 1;
    linkages(ipair).min = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ;0;0];
    linkages(ipair).max = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0; 0;0];
    %
end
%
% --------------------------------------------
% OCP PARAMETERS DEFINITION
% --------------------------------------------
%
setup.name        = 'Interplanetary';
setup.funcs.cost  = 'InterplanetaryCost';
setup.funcs.dae   = 'InterplanetaryDae';
setup.funcs.event = 'InterplanetaryEvent';
setup.funcs.link  = 'InterplanetaryLink';
setup.limits      = limits;
setup.derivatives = 'numerical';
setup.parallel    = 'yes';
setup.guess       = guess;
setup.linkages    = linkages;
setup.autoscale   = 'off';
setup.solver      = 'ipopt';
setup.method      = 'collocation';
%
if strcmp(setup.parallel,'yes') && strcmp(setup.derivatives,'numerical')
    %
    parfor i  = 1:4
        %
        load_spice_kernels('/Users/davidmorante/Desktop/MicePackage/');
        %
    end
end
%
% --------------------------------------------
% CALL THE SOLVER
% --------------------------------------------
%
%output   = DMG(setup);
%output = guess;
%solution = output.solution;
%solution = guess;
%
figure(3)
hold on
%%
solution(1).time = solution(1).time';
solution(2).time = solution(2).time';
save solution
%%
%
% Plot Trajectory
%
n_fb = numel(setup.flyby_planet);
%
figure(1)
hold on
grid on
axis equal
%
for i = 1:n_fb + 1
    %
    x     = solution(i).state(:,1);
    y     = solution(i).state(:,2);
    z     = solution(i).state(:,3);
    %
    plot3(x(1),y(1),z(1),'o')
    plot3(x(end),y(end),z(end),'o')
    %
    T     = solution(i).control(:,1);
    T     = T>0.4;
    %
    swicth_point = find(not(T(1:end-1) == T(2:end)));
    %
    swicth_point_ext = [1;swicth_point;numel(T) ];
    %
    
    for jj = 1:numel(swicth_point)+1
        ind1 = swicth_point_ext(jj);
        ind2 = swicth_point_ext(jj+1);
        plot3(x(ind1),y(ind1),z(ind1),'o')
        plot3(x(ind2),y(ind2),z(ind2),'o')
        switch T(ind1+1)
            
            case 0
                
                plot3(x(ind1:ind2),y(ind1:ind2),z(ind1:ind2),'. black')
                
            case 1
                
                plot3(x(ind1:ind2),y(ind1:ind2),z(ind1:ind2),'- black','LineWidth',1.5)
                
        end
    end
    
end
%
% Plot Planets Trajectory
%
planets = [setup.departure_planet, setup.flyby_planet, setup.arrival_planet];
%
t0 = solution(1).parameter(end);
%
et0_min    = cspice_str2et(setup.Initial_Date);
et0_max    = cspice_str2et(setup.Final_Date);
et0_factor = t0;
et0        = et0_min + (et0_max - et0_min)*et0_factor;
%
%
time = linspace(0,1,100);
%
x = zeros(numel(time));
y = zeros(numel(time));
z = zeros(numel(time));

for j = 1:n_fb + 2
    S1  = cspice_spkezr(planets{j}, et0 , 'ECLIPJ2000', 'NONE', 'SUN');
    rp  = S1(1:3)*1e3/setup.lc;
    T   = 3*pi*sqrt(norm(rp)^3)*setup.tc;
    
    for kk = 1:numel(time)
        S1  = cspice_spkezr(planets{j}, et0 + T*time(kk) , 'ECLIPJ2000', 'NONE', 'SUN');
        rp  = S1(1:3)*1e3/setup.lc;
        x(kk) = rp(1);
        y(kk) = rp(2);
        z(kk) = rp(3);
    end
    plot3(x,y,z);
end
%


%%
%
% Thrust Profile
%
vinf0    = solution(1).parameter(end-1)
launcher = setup.launcher;
%
m0 = launcher_model(launcher,vinf0^2)
%
% Launch declination and Right Ascension
%
alpha1 = solution(1).parameter(3)
alpha2 = solution(1).parameter(4)
vinf0  = [cos(alpha1)*cos(alpha2),cos(alpha1)*sin(alpha2),sin(alpha1)]
%
v_inf0x  = vinf0(1);
v_inf0y  = 0.91748206*vinf0(2) - 0.39777716*vinf0(3);
v_inf0z  = 0.39777716*vinf0(3) + 0.91748206*vinf0(2);
%
declination  = atan(v_inf0z/sqrt(v_inf0x^2+v_inf0y^2))*180/pi
%
% Obtener Altura de Flybys
%
rfb = zeros(1,n_fb);
for j = 1:n_fb
    rfb(j) = (solution(j).parameter(1))*1e6;
    if rfb(j) < 200
        rfb(j) = 200;
    end
end
Flyby_radiuses = rfb
%
% Obtener el beta angle del Flyby
%
cc = zeros(1,n_fb);
for j = 1:n_fb
    cc(j) = (solution(j).parameter(2));
end
declination = cc*180/pi
%
% Relative Velocity
%
planets = [setup.flyby_planet, setup.arrival_planet]
%
for j = 1:n_fb+1
    %
    et = et0 + solution(j).time(end)*setup.tc;
    %
    v = solution(j).state(end,4:6)';
    S1  = cspice_spkezr(planets{j}, et, 'ECLIPJ2000', 'NONE', 'SUN');
    vp = S1(4:6)*1e3/setup.vc;
    v_inf1  = v - vp;
    vinf(j) = norm(v_inf1)*setup.vc/1e3;
    %
end

relative_velocity = vinf

figure (2)
grid on
hold on
for j = 1:n_fb+1
    
    display(['Fase:',num2str(j)])
    time  = solution(j).time;
    mass  = solution(j).state(:,7);
    T     = solution(j).control(:,1);
    alpha = solution(j).control(:,2);
    beta  = solution(j).control(:,3);
    Isp   = solution(j).control(:,4)*1000;
    
    x     = solution(j).state(:,1);
    y     = solution(j).state(:,2);
    z     = solution(j).state(:,3);
    
    r = sqrt( x.^2 + y.^2 + z.^2 );
    
    P   = setup.P0;
    P_margin = 0.15;
    eta = 0.6;
    decay = 0.02; % percentage per year
    decay_factor = (1-decay).^(time*setup.tc/(365*24*3600));
    duty_cycle   = 0.9;
    %
    TT  =  duty_cycle*T.*(2*eta*(1-P_margin)*(decay_factor*1000-200))./(Isp*setup.g0);
    TT  = TT.*(TT>0);
    
    
    time       = time*setup.tc;
    et0_min    = cspice_str2et(setup.Initial_Date);
    et0_max    = cspice_str2et(setup.Final_Date);
    et0_factor = solution(j).parameter(end);
    et0        = et0_min + (et0_max - et0_min)*et0_factor;
    time       = et0 + time;
    
    dtime =  datetime('2000-01-01 00:00:00') + seconds(time);
    datetime(dtime);
    time1 = datetime(dtime(1))
    time2 = datetime(dtime(end))
    m1    = m0*mass(1)
    m2    = m0*mass(end)
    %
    % Plot flyby line
    %
    if j == 1
        plot([dtime(end),dtime(end)],[0,3.5])
        
        
    end
    %
   % if j ==1
   %     dtime2 =  datetime('2000-01-01 00:00:00') + seconds([time(1:8);time(14:end)]);
   % plot(dtime2,[T(1:8).*Isp(1:8);T(14:end).*Isp(14:end)]/1000)
   % plot(dtime2,[TT(1:8);TT(14:end)]*10)
   %    plot(dtime(14:end),T(14:end).*Isp(14:end)/1000)
   % else
           plot(dtime,TT*10)
    plot(dtime,T.*Isp/1000) 
   % end
    %NumTicks = 12;
    %L = get(gca,'XLim');
    %set(gca,'XTick',linspace(L(1),L(2),NumTicks))
      
end

    NumTicks = 20;
    L = get(gca,'XLim');
    set(gca,'XTick',linspace(L(1),L(2),NumTicks))

   datetick('x','mmm yyyy','keeplimits', 'keepticks')
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    xtickangle(45)

