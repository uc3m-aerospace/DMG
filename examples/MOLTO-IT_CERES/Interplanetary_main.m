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
m0        = 568; %Kg
P0        = 10;  %KW
v_inf0    = 1.6;
setup.g0  = 9.81;
setup.Isp = 3000;
setup.lc  = 149597870.700e03;
setup.mu  = 132712440018e09;
setup.tc  = sqrt(setup.lc^3/setup.mu);
setup.vc  = setup.lc/setup.tc;
setup.hpm = 562.05e3/setup.lc;
setup.ac  = setup.lc/setup.tc^2;
setup.m0  = m0;
setup.P0  = P0;
setup.v_inf0 = v_inf0; 
% --------------------------------------------
% MISSION SETUP
% --------------------------------------------
setup.departure_planet = {'Earth'};
setup.arrival_planet   = {'Ceres'};
setup.flyby_planet     = {'Mars'};
setup.Tmax             = 0.18/setup.ac/m0;
setup.Initial_Date     = '2003 Jan 2 00:00:00';
setup.Final_Date       = '2003 Dec 02 00:00:00';
%
% Get Initial Conditions
%
setup.et0 = cspice_str2et(setup.Initial_Date);
t0 = 0;
tfMin = 0;
tfMax1 = 2.71*365*24*3600/setup.tc;
%tfMax1 = 2.7123*365*24*3600/setup.tc;
%tfMax1 = 2.6575*365*24*3600/setup.tc;
%
% Limits
%
xmin  = -3;
xmax  = 3;
ymin  = -3; 
ymax  = 3;
zmin  = -1;
zmax  = 1;
vxmin = -3;
vxmax = 3;
vymin = -3;
vymax = 3;
vzmin = -3;
vzmax = 3;
%
thetamin = -pi;
thetamax = 3*pi;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LEG 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
iphase = 1;
% STATE VARIABLE
limits(iphase).nodes = 80;
limits(iphase).time.min = [t0 tfMin];
limits(iphase).time.max = [t0 tfMax1];
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
limits(iphase).state.min(7,:)   = [1 0.2 0.2];
limits(iphase).state.max(7,:)   = [1 1 1];
% CONTROL
limits(iphase).control.min(1,:) = 0;
limits(iphase).control.max(1,:) = 1;
limits(iphase).control.min(2,:) = -pi;
limits(iphase).control.max(2,:) = pi; % alpha (inplane-angle)
limits(iphase).control.min(3,:) = -pi/2;
limits(iphase).control.max(3,:) = pi/2; % beta (outplane-angle)
% PAREMETERS
limits(iphase).parameter.min    = [200/1000;-pi;-pi/2;-pi; 0];
limits(iphase).parameter.max    = [1000/1000;pi; pi/2; pi; 1];
% CONSTRAINTS LIMITS
limits(iphase).path.min    = [];
limits(iphase).path.max    = [];
limits(iphase).event.min   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
limits(iphase).event.max   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
% --------------------------------------------
% INITIAL GUESS PHASE 1
% --------------------------------------------
%
initial_guess_1 = load('Trajectory_Leg1.output');
%
t_guess     = initial_guess_1(:,1);
r_guess     = initial_guess_1(:,2);
v_guess     = initial_guess_1(:,3);
psi_guess   = initial_guess_1(:,4);
T_guess     = initial_guess_1(:,5);
T_guess1    = T_guess;
alpha_guess = initial_guess_1(:,6);
theta_guess = initial_guess_1(:,7);
m_guess     = 1 - mean(T_guess1)/(setup.g0/setup.ac*setup.Isp/setup.tc).*t_guess;
%
x_guess  = r_guess.*cos(theta_guess);
y_guess  = r_guess.*sin(theta_guess);
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
guess(iphase).control(:,1) = T_guess/setup.Tmax.*r_guess.^2.*m_guess;
guess(iphase).control(:,2) = alpha_guess;
guess(iphase).control(:,3) = 0;
guess(iphase).parameter    = [200;0;0;0;0.483152161027274];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LEG 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
iphase = 2;
limits(iphase).nodes = 80;
limits(iphase).time.min = [tfMin  tfMax1];
limits(iphase).time.max = [tfMax1 tfMax1];
% STATE
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
limits(iphase).state.min(7,:)   = [0.2 0.2 0.2];
limits(iphase).state.max(7,:)   = [1 1 1];
% CONTROL
limits(iphase).control.min(1,:) = 0;
limits(iphase).control.max(1,:) = 1;
limits(iphase).control.min(2,:) = -pi;
limits(iphase).control.max(2,:) =  pi; % alpha (inplane-angle)
limits(iphase).control.min(3,:) = -pi/2;
limits(iphase).control.max(3,:) = pi/2; % beta (outplane-angle)
% PARAMETERS
limits(iphase).parameter.min    = [0];
limits(iphase).parameter.max    = [1];
% CONSTRAINTS LIMITS
limits(iphase).path.min    = [];
limits(iphase).path.max    = [];
limits(iphase).event.min   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];
limits(iphase).event.max   = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ];

% --------------------------------------------
% LINKAGES CONSTRAINTS
% --------------------------------------------

ipair = 1;
linkages(ipair).left.phase = 1;
linkages(ipair).right.phase =2;
linkages(ipair).min = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
linkages(ipair).max = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
%
% --------------------------------------------
% INITIAL GUESS PHASE 2
% --------------------------------------------
%
initial_guess_2 = load('Trajectory_Leg2.output');
t_end       = t_guess(end);
t_guess     = initial_guess_2(:,1)+t_end;
r_guess     = initial_guess_2(:,2);
v_guess     = initial_guess_2(:,3);
psi_guess   = initial_guess_2(:,4);
T_guess     = initial_guess_2(:,5);
alpha_guess = initial_guess_2(:,6);
theta_guess = initial_guess_2(:,7);
m_guess     = m_guess(end) - mean(T_guess)/(setup.g0/setup.ac*setup.Isp/setup.tc).*(t_guess-t_end);
%
x_guess  = r_guess.*cos(theta_guess);
y_guess  = r_guess.*sin(theta_guess);
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
guess(iphase).control(:,1) = T_guess/setup.Tmax.*r_guess.^2.*m_guess;
guess(iphase).control(:,2) = alpha_guess;
guess(iphase).control(:,3) = 0;
guess(iphase).parameter    = [0.483152161027274];

% --------------------------------------------
% OCP PARAMETERS DEFINITION
% --------------------------------------------
%guess = solution;
setup.name = 'Interplanetary';
setup.funcs.cost = 'InterplanetaryCost';
setup.funcs.dae = 'InterplanetaryDae';
setup.funcs.event = 'InterplanetaryEvent';
setup.funcs.link = 'InterplanetaryLink';
setup.limits = limits;
setup.derivatives = 'numerical';
setup.parallel = 'no';
setup.guess = guess;
setup.linkages = linkages;
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='collocation';

% --------------------------------------------
% CALL THE SOLVER
% --------------------------------------------

 %output = DMG(setup);
 %solution = output.solution;%
 %solution(1).time = solution(1).time';
 %solution(2).time = solution(2).time';
 %save solution
 solution = guess;
 %% Plot solution Earth Ceres Mars
x     = [ solution(1).state(:,1);solution(2).state(:,1)];
y     = [ solution(1).state(:,2);solution(2).state(:,2)];
z     = [ solution(1).state(:,3);solution(2).state(:,3)];
timet = [ solution(1).time;solution(2).time];
 
plot3(x,y,z)
 
t0 = solution(1).parameter(end);
%
et0_min    = cspice_str2et(setup.Initial_Date);
et0_max    = cspice_str2et(setup.Final_Date);
et0_factor = t0;
et0        = et0_min + (et0_max - et0_min)*et0_factor;
time       = et0 + timet*setup.tc;
    
dtime =  datetime('2000-01-01 12:00:00') + seconds(time);
datetime(dtime);
time1 = datetime(dtime(1))
time2 = datetime(dtime(end))
MJD   = juliandate(dtime)

%jdtdb = tdbjuliandate(MJD)
  
A = [MJD,x*setup.lc/1000,y*setup.lc/1000,z*setup.lc/1000];
 
dlmwrite('EMC.txt',A, 'precision', 15, 'delimiter','\t')
 
axis equal

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
t0 = solution(1).parameter(end);
%
et0_min    = cspice_str2et(setup.Initial_Date);
et0_max    = cspice_str2et(setup.Final_Date);
et0_factor = t0;
et0        = et0_min + (et0_max - et0_min)*et0_factor;
%
n_fb = 1;
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
                dtime =  datetime('2000-01-01 12:00:00') +seconds(et0 + time(ind1:ind2));
                datetime(dtime)
                xx = x(ind1:ind2);
                yy = y(ind1:ind2);
                zz = z(ind1:ind2);
                plot3(xx,yy,zz,'. black')
                MJD   = juliandate(dtime)
                
                A = [MJD,xx*setup.lc/1000,yy*setup.lc/1000,zz*setup.lc/1000];
                
                dlmwrite('EMC_coast2.txt',A, 'precision', 15, 'delimiter','\t')
 
                
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
 
 
%control = [guess(1).control(:,1),guess(2).control(:,1)];
%theta   = [guess(1).state(:,4),guess(2).state(:,4)];
%r       = [guess(1).state(:,1),guess(2).state(:,1)];
%m       = [guess(1).state(:,5),guess(2).state(:,5)];
%alpha   = [guess(1).control(:,2),guess(2).control(:,2)];
% %
% figure(2)
% %plot(theta,control*setup.Tmax./r.^2./m*setup.ac)
% %hold on
% 
% % figure(3)
% % plot(theta,alpha)
% % hold on
% %%
% solution = solution;
% control = [solution(1).control(:,1),solution(2).control(:,1)]
% x     = [ solution(1).state(:,1),solution(2).state(:,1)];
% y     = [ solution(1).state(:,2),solution(2).state(:,2)];
% z     = [ solution(1).state(:,3),solution(2).state(:,3)];
% %
% 
% m     = [ solution(1).state(:,7),solution(2).state(:,7)];
% figure(1)
% plot3(x,y,z)
% hold on
% axis equal
% grid on
% 
% 
%  et0              = setup.et0;
%  time = linspace(et0,et0+tfMax1*setup.tc*2,100);
%  r_ceres = zeros(100,3);
%  for i = 1:100
%  S0  = cspice_spkezr('Ceres', time(i), 'ECLIPJ2000', 'NONE', 'SUN');
%  r_ceres(i,:) = S0(1:3)*1e3/setup.lc;
%  end
%  
%  time = linspace(et0,et0+tfMax1*setup.tc,100);
%  r_earth = zeros(100,3);
%  for i = 1:100
%  S0  = cspice_spkezr('Earth', time(i), 'ECLIPJ2000', 'NONE', 'SUN');
%  r_earth(i,:) = S0(1:3)*1e3/setup.lc;
%  end
%  
%  time = linspace(et0,et0+tfMax1*setup.tc,100);
%  r_mars = zeros(100,3);
%  for i = 1:100
%  S0  = cspice_spkezr('Mars', time(i), 'ECLIPJ2000', 'NONE', 'SUN');
%  r_mars(i,:) = S0(1:3)*1e3/setup.lc;
%  end
%  
% 
% plot3(r_ceres(:,1),r_ceres(:,2),r_ceres(:,3))
% plot3(r_earth(:,1),r_earth(:,2),r_earth(:,3))
% plot3(r_mars(:,1),r_mars(:,2),r_mars(:,3))


% %
% r = sqrt(x.^2+y.^2+z.^2);
% %
% control = [solution(1).control(:,1),solution(2).control(:,1)];
% alpha   = [solution(1).control(:,2),solution(2).control(:,2)];
% beta    = [solution(1).control(:,3),solution(2).control(:,3)];
% time    = [solution(1).time(:),solution(2).time(:)];
% P0 = 10;
% P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r);
% P = 2.6 *(P>2.6) + P.*(P<=2.6) ;
% TT = control.*(-1.9137 + 36.242 * P)/(568*setup.ac)/1000* (P>0.649);
% %
% figure(2)
% plot(time/(24*3600)*setup.tc,TT./m*setup.ac)
% %
% figure(3)
% plot(time/(24*3600)*setup.tc,beta*180/pi)
%
%objective = 1 - exp( - 2.4528346470712592e-01*setup.vc /3000/9.81)
%
%%
sol.left.state = solution(1).state(end,:);
sol.left.time = solution(1).time(end);
sol.right.time = solution(2).time(end);
sol.left.parameter = solution(1).parameter ;
sol.left.phase = 1;
sol.right.state = solution(2).state(end,:);
sol.right.parameter =solution(2).parameter ;
sol.right.phase = 2;
% %'
 InterplanetaryLink(sol,setup)


