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
setup.flyby_planet     = [{'Mars'}];
setup.last_phase = 2;
setup.Tmax             = 0.18/setup.ac/m0;
setup.t0               = 0.131662;
setup.Initial_Date     = '2029 Jan 1 00:00:00';
setup.Final_Date       = '2030 Dec 31 00:00:00';

%
% Get Initial Conditions
%
setup.et0 = cspice_str2et(Initial_Date);
t0 = 0;
tfMin = 0;
tfMax1 = 3.9*365*24*3600/setup.tc;
%tfMax1 = 23.11515010;
%
% Limits
%
xmin  = -7;
xmax  = 7;
ymin  = -7; 
ymax  = 7;
zmin  = -1;
zmax  = 1;
vxmin = -2;
vxmax = 2;
vymin = -2;
vymax = 2;
vzmin = -1;
vzmax = 1;
%
hpmin   = [200 200 200];
hpmax   = [300000 120000 100000];
hpguess = [200 200 200]; 
c       = [0 0 0];
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
nnodes = [60,60];
for iphase = 1:setup.last_phase
% STATE VARIABL5
limits(iphase).nodes = nnodes(iphase);
limits(iphase).time.min = [t0 tfMin];
if iphase == 1
limits(iphase).time.max = [t0 tfMax1];
elseif iphase == setup.last_phase
 limits(iphase).time.min = [t0 tfMin]; 
 limits(iphase).time.max = [tfMax1 tfMax1];
else 
limits(iphase).time.max = [tfMax1 tfMax1];
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
limits(iphase).parameter.min    = [hpmin(iphase)/1000;-pi;-pi/2;-pi; 0];
limits(iphase).parameter.max    = [hpmax(iphase)/1000;pi; pi/2; pi;  1];
elseif iphase < setup.last_phase
limits(iphase).parameter.min    = [hpmin(iphase)/1000;-pi;0];
limits(iphase).parameter.max    = [hpmax(iphase)/1000;pi; 1];
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
% %
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
% guess(iphase).parameter    = [hpguess(iphase);c(iphase);0;-pi/4;setup.t0];
% elseif iphase < setup.last_phase
% guess(iphase).parameter    = [hpguess(iphase);c(iphase);setup.t0];
% else
% guess(iphase).parameter    = [setup.t0];
% end
% 
 end

guess = solution;
% --------------------------------------------
% LINKAGES CONSTRAINTS
% --------------------------------------------
ipair = 1;
linkages(ipair).left.phase = 1;
linkages(ipair).right.phase =2;
linkages(ipair).min = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0; 0 ];
linkages(ipair).max = [ 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0; 0 ];
%
%%

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
setup.method ='hermite-simpson';
%
if strcmp(setup.parallel,'yes') && strcmp(setup.derivatives,'numerical')
    %
    parfor i  = 1:4
        %
        load_spice_kernels('/Users/davidmorante/Desktop/MicePackage/');
        %
    end
end
% --------------------------------------------
% CALL THE SOLVER
% --------------------------------------------

output = DMG(setup);
solution = output.solution;%

solution(1).time = solution(1).time';
solution(2).time = solution(2).time';
 
save solution
% control = [guess(1).control(:,1),guess(2).control(:,1)];
% theta   = [guess(1).state(:,4),guess(2).state(:,4)];
% r       = [guess(1).state(:,1),guess(2).state(:,1)];
% m       = [guess(1).state(:,5),guess(2).state(:,5)];
% alpha   = [guess(1).control(:,2),guess(2).control(:,2)];
%
figure(2)
%plot(theta,control*setup.Tmax./r.^2./m*setup.ac)
%hold on

figure(3)
%plot(theta,alpha)
hold on
%%
solution = solution;
time   = [[ solution(1).time;solution(2).time(2:end)]];
 x     = [ solution(1).state(:,1);solution(2).state(:,1)];
 y     = [ solution(1).state(:,2);solution(2).state(:,2)];
 z     = [ solution(1).state(:,3);solution(2).state(:,3)];
 
 %x =interp1(time,x(1:end-1), linspace(time(1),time(end),1000),'spline');
 %y =interp1(time,y(1:end-1), linspace(time(1),time(end),1000),'spline');
 %z =interp1(time,z(1:end-1), linspace(time(1),time(end),1000),'spline');
% %
% 
% m     = [ solution(1).state(:,7),solution(2).state(:,7),solution(3).state(:,7),solution(4).state(:,7)];
 %figure(2)
 plot3(x,y,z)
 axis equal
 grid on
% %
% r = sqrt(x.^2+y.^2+z.^2);
% %
% control = [solution(1).control(:,1),solution(2).control(:,1),solution(3).control(:,1),solution(4).control(:,1)];
% %alpha   = [solution(1).control(:,2),solution(2).control(:,3),solution(3).control(:,4),solution(4).control(:,5)];
% alpha   = [solution(1).control(:,2),solution(2).control(:,2),solution(3).control(:,2),solution(4).control(:,2)];
% time    = [solution(1).time(:),solution(2).time(:),solution(3).time(:),solution(4).time(:)];
% P0 = 10;
% P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r)
% P = 2.6 *(P>2.6) + P.*(P<=2.6) ;
% TT = control.*(-1.9137 + 36.242 * P)/(300*setup.ac)/1000.* (P>0.649);
% %
% figure(2)
% plot(time/(24*3600)*setup.tc,TT./m*setup.ac)
% %
% figure(3)
% plot(time,alpha)
% %
%objective = 1 - exp( - 2.4528346470712592e-01*setup.vc /3000/9.81)
%
 sol.left.state = solution(4).state(end,:);
 sol.left.time = solution(4).time(end);
 sol.left.parameter = solution(3).parameter ;
 sol.left.phase = 4;
 sol.right.time = solution(4).time(end);
 sol.right.state = solution(4).state(end,:);
 sol.right.parameter =solution(3).parameter ;
 sol.right.phase = 4;
% %'
 InterplanetaryLink(sol,setup)


