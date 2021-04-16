% --------------------------------------------
% Interplanetary Trajectories Problem
% --------------------------------------------
clear setup limits guess linkages
%
addpath('/Users/davidmorante/Desktop/MicePackage')
EM  = load_spice_kernels('/Users/davidmorante/Desktop/MicePackage/');
% --------------------------------------------
% CONSTANTS SETUP
% -------------------------------------------
m0        = 300; %Kg
setup.P_available = 2.6;
P0        = 10;  %KW
v_inf0    = 2;
setup.g0  = 9.81;
setup.Isp = 3000;
setup.lc  = 149597870.700e03;
setup.mu  = 132712440018e09;
setup.tc  = sqrt(setup.lc^3/setup.mu);
setup.vc  = setup.lc/setup.tc;
setup.ac  = setup.lc/setup.tc^2;
setup.m0  = m0;
setup.P0  = P0;
setup.v_inf0 = v_inf0; 
% --------------------------------------------
% MISSION SETUP
% --------------------------------------------
setup.departure_planet = {'Earth'};
setup.arrival_planet   = {'Jupiter'};
setup.flyby_planet     = [{'Venus'},{'Mars'}];
setup.last_phase = 3;
setup.Tmax             = 0.18/setup.ac/m0;
%
setup.t0               = 0.4;
setup.Initial_Date     = '2029 Jan 1 00:00:00';
setup.Final_Date       = '2030 Dec 31 00:00:00';

% setup.Initial_Date     = '2029 Oct 1 00:00:00';
% setup.Final_Date       = '2029 Oct 1 00:00:00';
%
% Get Initial Conditions
%
setup.et0 = cspice_str2et(Initial_Date);
t0 = 0;
tfMin = 0;
tfMax1 = 2.9*365*24*3600/setup.tc;
%tfMax1 = 23.11515010;
%
% Limits
%
xmin  = -6;
xmax  = 6;
ymin  = -6; 
ymax  = 6;
zmin  = -6;
zmax  = 6;
vxmin = -3;
vxmax = 3;
vymin = -3;
vymax = 3;
vzmin = -3;
vzmax = 3;
%
hpmin   = [200 200 200];
hpmax   = [300000 120000 100000];
hpguess = [200 200 200]; 
c       = [-pi 0 0];
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
for iphase = 1:3
% STATE VARIABLE
limits(iphase).nodes = 70;
limits(iphase).time.min = [t0 tfMin];
if iphase == 1
limits(iphase).time.max = [t0 tfMax1];
else
limits(iphase).time.max = [tfMax1 tfMax1];
end
if iphase == 3
 limits(iphase).time.min = [t0 tfMax1];   
end
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
limits(iphase).state.min(7,:)   = [1 0.01 0.01];
else
limits(iphase).state.min(7,:)   = [0.01 0.01 0.01];
end
limits(iphase).state.max(7,:)   = [1 1 1];
% CONTROL
limits(iphase).control.min(1,:) = 0;
limits(iphase).control.max(1,:) = 1;
limits(iphase).control.min(2,:) = -pi;
limits(iphase).control.max(2,:) = pi; % alpha (inplane-angle)
limits(iphase).control.min(3,:) = -pi/2;
limits(iphase).control.max(3,:) = pi/2; % beta (outplane-angle)
% PAREMETERS
if iphase == 1
limits(iphase).parameter.min    = [hpmin(iphase)/1000;-pi;-pi/2;-pi;0];
limits(iphase).parameter.max    = [hpmax(iphase)/1000;pi; pi/2; pi;1];
elseif iphase < 3
limits(iphase).parameter.min    = [hpmin(iphase)/1000;-pi;0];
limits(iphase).parameter.max    = [hpmax(iphase)/1000;pi;1];
else
limits(iphase).parameter.min    = [0];
limits(iphase).parameter.max    = [1];    
end
% CONSTRAINTS LIMITS
limits(iphase).path.min    = [];
limits(iphase).path.max    = [];
if iphase == 1
limits(iphase).event.min   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
limits(iphase).event.max   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
else
limits(iphase).event.min   = [ 0 ; 0 ; 0 ];
limits(iphase).event.max   = [ 0 ; 0 ; 0 ];
end
% % --------------------------------------------
% % INITIAL GUESS PHASE 1
% % --------------------------------------------
%
% initial_guess_1 = load(['Trajectory_Leg',num2str(iphase),'.output']);
% %
% t_guess     = initial_guess_1(:,1);
% r_guess     = initial_guess_1(:,2);
% v_guess     = initial_guess_1(:,3);
% psi_guess   = initial_guess_1(:,4);
% T_guess     = initial_guess_1(:,5)/10;
% alpha_guess = initial_guess_1(:,6);
% theta_guess = initial_guess_1(:,7);
% m_guess     = m_0*exp(-mean(T_guess)/(setup.g0/setup.ac*setup.Isp/setup.tc).*(t_guess-t_guess(1)));
% %
% m_0 = m_guess(end);
% %
% x_guess  = r_guess.*cos(theta_guess);
% y_guess  = r_guess.*sin(theta_guess);
% vx_guess = v_guess.* ( cos(psi_guess).*cos(theta_guess) - sin(psi_guess).*sin(theta_guess) );
% vy_guess = v_guess.* ( cos(psi_guess).*sin(theta_guess) + sin(psi_guess).*cos(theta_guess) );
% %
% guess(iphase).time         = t_guess;
% guess(iphase).state(:,1)   = x_guess;
% guess(iphase).state(:,2)   = y_guess;
% guess(iphase).state(:,3)   = 0;
% guess(iphase).state(:,4)   = vx_guess;
% guess(iphase).state(:,5)   = vy_guess;
% guess(iphase).state(:,6)   = 0;
% guess(iphase).state(:,7)   = m_guess;
% guess(iphase).control(:,1) = T_guess/setup.Tmax.*r_guess.^2.*m_guess;
% guess(iphase).control(:,2) = alpha_guess;
% guess(iphase).control(:,3) = 0;
% %
% if iphase == 1
% guess(iphase).parameter    = [hpguess(iphase);c(iphase);0;-pi/4; setup.t0];
% elseif iphase < 3
% guess(iphase).parameter    = [hpguess(iphase);c(iphase);setup.t0];
% else
% guess(iphase).parameter    = [setup.t0];
% end
% % 
  end
guess = solution;
% --------------------------------------------
% LINKAGES CONSTRAINTS
% --------------------------------------------
ipair = 1;
linkages(ipair).left.phase = 1;
linkages(ipair).right.phase =2;
linkages(ipair).min = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
linkages(ipair).max = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
%
ipair = 2;
linkages(ipair).left.phase  = 2;
linkages(ipair).right.phase = 3;
linkages(ipair).min = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
linkages(ipair).max = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
%
%
%%
% --------------------------------------------
% OCP PARAMETERS DEFINITION
% --------------------------------------------

setup.name = 'Interplanetary';
setup.funcs.cost = 'InterplanetaryCost';
setup.funcs.dae = 'InterplanetaryDae';
setup.funcs.event = 'InterplanetaryEvent';
setup.funcs.link = 'InterplanetaryLink';
setup.limits = limits;
setup.derivatives = 'numerical';
setup.parallel = 'yes';
setup.guess = guess;
setup.linkages = linkages;
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='collocation';
if strcmp(setup.parallel,'yes');
    
end
% --------------------------------------------
% CALL THE SOLVER
% --------------------------------------------

output = DMG(setup);
solution = output.solution;%

solution(1).time = solution(1).time';
solution(2).time = solution(2).time';
solution(3).time = solution(3).time';

save solution
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
t0 = solution(1).parameter(end);
%
et0_min    = cspice_str2et(setup.Initial_Date);
et0_max    = cspice_str2et(setup.Final_Date);
et0_factor = t0;
et0        = et0_min + (et0_max - et0_min)*et0_factor;
%
for i = 1:n_fb + 1
    %
    x     = solution(i).state(:,1);
    y     = solution(i).state(:,2);
    z     = solution(i).state(:,3);
    time  = solution(i).time*setup.tc;
    mass  = solution(i).state(:,7);
    %
    plot3(x(1),y(1),z(1),'o')
    plot3(x(end),y(end),z(end),'o')
    %
    T     = solution(i).control(:,1);
    T     = T>0.7;
    %
    swicth_point = find(not(T(1:end-1) == T(2:end)))
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
    
    time       = et0 + time;
    
    dtime =  datetime('2000-01-01 00:00:00') + seconds(time);
    datetime(dtime);
    time1 = datetime(dtime(1))
    time2 = datetime(dtime(end))
    m1    = m0*mass(1)
    m2    = m0*mass(end)
    
end
%
% Plot Planets Trajectory
%
planets = [setup.departure_planet, setup.flyby_planet, setup.arrival_planet];
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
% Obtener Altura de Flybys
%
rfb = zeros(1,n_fb);
for j = 1:n_fb
    rfb(j) = (solution(j).parameter(1))*1e3;
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
 % Relative velocity
 %
 for j = 1:n_fb+1
    %
    et = et0 + solution(j).time(end)*setup.tc;
    %
    v = solution(j).state(end,4:6)';
    planets{j}
    S1  = cspice_spkezr(planets{j+1}, et, 'ECLIPJ2000', 'NONE', 'SUN');
    vp = S1(4:6)*1e3/setup.vc;
    v_inf1  = v - vp;
    vinf(j) = norm(v_inf1(1:3))*setup.vc/1e3;
    %
 end
vinf




