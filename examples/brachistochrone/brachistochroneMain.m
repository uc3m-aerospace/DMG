% -----------------------
% Brachistochrone Problem
% -----------------------
% --------------------------------------------------
% This example is a modification of the problem found
% in the following reference:
% --------------------------------------------------
% Garg, D., Patterson, M. A., Hager, W. W., Rao, A. V., 
% Benson, D. A., and Huntington, G. T., "A Unified Framework
% for the Numerical Solution of Optimal Control Problems Using
% Pseudospectral Methods," Working Manuscript, http://vdol.mae.ufl.edu
%  --------------------------------------------------
% The problem solved here is given as follows:
% Minimize 
%      t_f
% subject to the dynamic constraints
%      dx/dt = v*sin(u)
%      dy/dt = -v*cos(u)
%      dv/dt = g*cos(u)
% with the boundary conditions
%      x(0) = 0, y(0) = 0, v(0) = 0
%      x(t_f) = 2, y(t_f) = -2, v(t_f) = FREE
%
%  --------------------------------------------------

clear setup phases guess


CONSTANTS.g = 10;
x0 = 0;
y0 = 0;
v0 = 0;
xf = 2;
yf = -2;
xmin = -50;
xmax =  50;
ymin = -50;
ymax =   0;
vmin = xmin;
vmax = xmax;
param_min = [];
param_max = [];
path_min = [];
path_max = [];
event_min = [];
event_max = [];
duration_min = [];
duration_max = [];

iphase = 1;
limits(iphase).nodes = 50;
limits(iphase).time.min = [0 0];
limits(iphase).time.max = [0 100];
limits(iphase).state.min(1,:) = [x0 xmin xf];
limits(iphase).state.max(1,:) = [x0 xmax xf];
limits(iphase).state.min(2,:) = [y0 ymin yf];
limits(iphase).state.max(2,:) = [y0 ymax yf];
limits(iphase).state.min(3,:) = [v0 vmin vmin];
limits(iphase).state.max(3,:) = [v0 vmax vmax];
limits(iphase).control.min    = -4*pi;
limits(iphase).control.max    =  4*pi;
limits(iphase).parameter.min  = param_min;
limits(iphase).parameter.max  = param_max;
limits(iphase).path.min       = path_min;
limits(iphase).path.max       = path_max;
limits(iphase).event.min      = event_min;
limits(iphase).event.max      = event_max;
limits(iphase).duration.min    = duration_min;
limits(iphase).duration.max    = duration_max;
guess(iphase).time             = [0; 10];
guess(iphase).state(:,1)      = [x0; xf];
guess(iphase).state(:,2)      = [y0; yf];
guess(iphase).state(:,3)      = [v0; 6];
guess(iphase).control         = [0; 0];
guess(iphase).parameter       = [];

setup.name  = 'Brachistochrone-Problem';
setup.funcs.cost = 'brachistochroneCost';
setup.funcs.dae = 'brachistochroneDae';
setup.limits = limits;
setup.guess = guess;
setup.linkages = [];
setup.derivatives = 'numerical';
setup.parallel    = 'no';
setup.autoscale = 'off';
setup.parallel  = 'no';
setup.solver ='ipopt';
setup.method ='pseudospectral';

setup.CONSTANTS = CONSTANTS;
output = DMG(setup);
solution = output.solution;

if 0,
t = solution.time;
x = solution.state(:,1);
u = solution.state(:,2);
y = u.^2;

% Plot Solution
figure(1);
pp = plot(t,x,'-bo',t,y,'-rd');
xl = xlabel('Time');
yl = ylabel('State');
ll = legend('x(t)','y(t)','Location','Best');
set(pp,'LineWidth',2);
set(xl,'FontName','Times','FontSize',16);
set(yl,'FontName','Times','FontSize',16);
set(gca,'FontName','Times','FontSize',16);
grid on;

figure(2);
pp = plot(x,y);
xl = xlabel('x(t)');
yl = ylabel('y(t)');
set(pp,'LineWidth',2);
set(xl,'FontName','Times','FontSize',16);
set(yl,'FontName','Times','FontSize',16);
set(gca,'FontName','Times','FontSize',16);
grid on;
end;

