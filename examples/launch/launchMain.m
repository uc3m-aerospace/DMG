% --------------------------------------------
% Multiple-Stage Launch Vehicle Ascent Example
% --------------------------------------------
% -----------------------------------------------------------------------
% This example can be found in one of the following three references:
%   Benson, D. A., A Gauss Pseudospectral Transcription for Optimal
%   Control, Ph.D. Thesis, Department of Aeronautics and
%   Astronautics, Massachusetts Institute of Technology, November 2004.
%
%   Huntington, G. T. Advancement and Analysis of a Gauss
%   Pseudospectral Transcription for Optimal Control, Ph.D. Thesis,
%   Department of Aeronautics and Astronautics, Massachusetts
%   Institute of Technology, May 2007.
%
%   Huntington, G. T., Benson, D. A., Kanizay, N., Darby, C. L.,
%   How, J. P., and Rao, A. V., "Computation of Boundary Controls
%   Using a Gauss Pseudospectral Method," 2007 Astrodynamics
%   Specialist Conference, Mackinac Island, Michigan, August 19-23, 2007.
% -----------------------------------------------------------------------
clear setup limits guess linkages

global CONSTANTS

omega            = 7.29211585e-5; % Earth rotation rate (rad/s)
omega_matrix  = [0 -omega 0; omega 0 0; 0 0 0]; 
CONSTANTS.omega_matrix = omega_matrix; % Rotation rate matrix (rad/s)  
CONSTANTS.mu = 3.986012e14;       % Gravitational parameter (m^3/s^2)
CONSTANTS.cd = 0.5;               % Drag coefficient
CONSTANTS.sa = 4*pi;              % Surface area (m^2)
CONSTANTS.rho0 = 1.225;           % sea level gravity (kg/m^3)
CONSTANTS.H = 7200.0;             % Density scale height (m)       
CONSTANTS.Re = 6378145.0;         % Radius of earth (m)
CONSTANTS.g0 = 9.80665;           % sea level gravity (m/s^2)

lat0 = 28.5*pi/180;               % Geocentric Latitude of Cape Canaveral
x0 = CONSTANTS.Re*cos(lat0);      % x component of initial position
z0 = CONSTANTS.Re*sin(lat0);      % z component of initial position
y0 = 0;
r0 = [x0; y0; z0];
v0 = CONSTANTS.omega_matrix*r0;

bt_srb = 75.2;
bt_first = 261;
bt_second = 700;

t0 = 0;
t1 = 75.2;
t2 = 150.4;
t3 = 261;
t4 = 961;

m_tot_srb     = 19290;
m_prop_srb    = 17010;
m_dry_srb     = m_tot_srb-m_prop_srb;
m_tot_first   = 104380;
m_prop_first  = 95550;
m_dry_first   = m_tot_first-m_prop_first;
m_tot_second  = 19300;
m_prop_second = 16820;
m_dry_second  = m_tot_second-m_prop_second;
m_payload     = 4164;
thrust_srb    = 628500;
thrust_first  = 1083100;
thrust_second = 110094;
mdot_srb      = m_prop_srb/bt_srb;
ISP_srb       = thrust_srb/(CONSTANTS.g0*mdot_srb);
mdot_first    = m_prop_first/bt_first;
ISP_first     = thrust_first/(CONSTANTS.g0*mdot_first);
mdot_second   = m_prop_second/bt_second;
ISP_second     = thrust_second/(CONSTANTS.g0*mdot_second);

af = 24361140;
ef = 0.7308;
incf = 28.5*pi/180;
Omf = 269.8*pi/180;
omf = 130.5*pi/180;
nuguess = 0;
cosincf = cos(incf);
cosOmf = cos(Omf);
cosomf = cos(omf);
oe = [af ef incf Omf omf nuguess];
[rout,vout] = launchoe2rv(oe,CONSTANTS.mu);
rout = rout';
vout = vout';

m10 = m_payload+m_tot_second+m_tot_first+9*m_tot_srb;
m1f = m10-(6*mdot_srb+mdot_first)*t1;
m20 = m1f-6*m_dry_srb;
m2f = m20-(3*mdot_srb+mdot_first)*(t2-t1);
m30 = m2f-3*m_dry_srb;
m3f = m30-mdot_first*(t3-t2);
m40 = m3f-m_dry_first;
m4f = m_payload;

CONSTANTS.thrust_srb    = thrust_srb;
CONSTANTS.thrust_first  = thrust_first;
CONSTANTS.thrust_second = thrust_second;
CONSTANTS.ISP_srb       = ISP_srb;
CONSTANTS.ISP_first     = ISP_first;
CONSTANTS.ISP_second    = ISP_second;

rmin = -2*CONSTANTS.Re;
rmax = -rmin;
vmin = -10000;
vmax = -vmin;
nodes = 3;

iphase = 1;
limits(iphase).nodes = nodes;
limits(iphase).time.min = [t0 t1];
limits(iphase).time.max = [t0 t1];
limits(iphase).state.min(1,:) = [r0(1) rmin rmin];
limits(iphase).state.max(1,:) = [r0(1) rmax rmax];
limits(iphase).state.min(2,:) = [r0(2) rmin rmin];
limits(iphase).state.max(2,:) = [r0(2) rmax rmax];
limits(iphase).state.min(3,:) = [r0(3) rmin rmin];
limits(iphase).state.max(3,:) = [r0(3) rmax rmax];
limits(iphase).state.min(4,:) = [v0(1) vmin vmin];
limits(iphase).state.max(4,:) = [v0(1) vmax vmax];
limits(iphase).state.min(5,:) = [v0(2) vmin vmin];
limits(iphase).state.max(5,:) = [v0(2) vmax vmax];
limits(iphase).state.min(6,:) = [v0(3) vmin vmin];
limits(iphase).state.max(6,:) = [v0(3) vmax vmax];
limits(iphase).state.min(7,:) = [m10 m1f m1f];
limits(iphase).state.max(7,:) = [m10 m10 m10];
limits(iphase).control.min(1,:) = -1;
limits(iphase).control.max(1,:) =  1;
limits(iphase).control.min(2,:) = -1;
limits(iphase).control.max(2,:) =  1;
limits(iphase).control.min(3,:) = -1;
limits(iphase).control.max(3,:) =  1;
limits(iphase).parameter.min    = [];
limits(iphase).parameter.max    = [];
limits(iphase).path.min    = 1;
limits(iphase).path.max    = 1;
guess(iphase).time = [t0; t1];
guess(iphase).state(:,1) = [r0(1); r0(1)];
guess(iphase).state(:,2) = [r0(2); r0(2)];
guess(iphase).state(:,3) = [r0(3); r0(3)];
guess(iphase).state(:,4) = [v0(1); v0(1)];
guess(iphase).state(:,5) = [v0(2); v0(2)];
guess(iphase).state(:,6) = [v0(3); v0(3)];
guess(iphase).state(:,7) = [m10; m1f];
guess(iphase).control(:,1) = [1; 1];
guess(iphase).control(:,2) = [0; 0];
guess(iphase).control(:,3) = [0; 0];
guess(iphase).parameter    = [];

iphase = 2;
limits(iphase).nodes = nodes;
limits(iphase).time.min    = [t1 t2];
limits(iphase).time.max    = [t1 t2];
limits(iphase).state.min(1,:) = [rmin rmin rmin];
limits(iphase).state.max(1,:) = [rmax rmax rmax];
limits(iphase).state.min(2,:) = [rmin rmin rmin];
limits(iphase).state.max(2,:) = [rmax rmax rmax];
limits(iphase).state.min(3,:) = [rmin rmin rmin];
limits(iphase).state.max(3,:) = [rmax rmax rmax];
limits(iphase).state.min(4,:) = [vmin vmin vmin];
limits(iphase).state.max(4,:) = [vmax vmax vmax];
limits(iphase).state.min(5,:) = [vmin vmin vmin];
limits(iphase).state.max(5,:) = [vmax vmax vmax];
limits(iphase).state.min(6,:) = [vmin vmin vmin];
limits(iphase).state.max(6,:) = [vmax vmax vmax];
limits(iphase).state.min(7,:) = [m2f m2f m2f];
limits(iphase).state.max(7,:) = [m20  m20  m20];
limits(iphase).control.min(1,:) = -1;
limits(iphase).control.max(1,:) =  1;
limits(iphase).control.min(2,:) = -1;
limits(iphase).control.max(2,:) =  1;
limits(iphase).control.min(3,:) = -1;
limits(iphase).control.max(3,:) =  1;
limits(iphase).parameter.min    = [];
limits(iphase).parameter.max    = [];
limits(iphase).path.min    = 1;
limits(iphase).path.max    = 1;
guess(iphase).time = [t1; t2];
guess(iphase).state(:,1) = [r0(1); r0(1)];
guess(iphase).state(:,2) = [r0(2); r0(2)];
guess(iphase).state(:,3) = [r0(3); r0(3)];
guess(iphase).state(:,4) = [v0(1); v0(1)];
guess(iphase).state(:,5) = [v0(2); v0(2)];
guess(iphase).state(:,6) = [v0(3); v0(3)];
guess(iphase).state(:,7) = [m20; m2f];
guess(iphase).control(:,1) = [1; 1];
guess(iphase).control(:,2) = [0; 0];
guess(iphase).control(:,3) = [0; 0];
guess(iphase).parameter    = [];

iphase = 3;
limits(iphase).nodes = nodes;
limits(iphase).time.min = [t2 t3];
limits(iphase).time.max = [t2 t3];
limits(iphase).state.min(1,:) = [rmin rmin rmin];
limits(iphase).state.max(1,:) = [rmax rmax rmax];
limits(iphase).state.min(2,:) = [rmin rmin rmin];
limits(iphase).state.max(2,:) = [rmax rmax rmax];
limits(iphase).state.min(3,:) = [rmin rmin rmin];
limits(iphase).state.max(3,:) = [rmax rmax rmax];
limits(iphase).state.min(4,:) = [vmin vmin vmin];
limits(iphase).state.max(4,:) = [vmax vmax vmax];
limits(iphase).state.min(5,:) = [vmin vmin vmin];
limits(iphase).state.max(5,:) = [vmax vmax vmax];
limits(iphase).state.min(6,:) = [vmin vmin vmin];
limits(iphase).state.max(6,:) = [vmax vmax vmax];
limits(iphase).state.min(7,:) = [m3f m3f m3f];
limits(iphase).state.max(7,:) = [m30  m30  m30];
limits(iphase).control.min(1,:) = -1;
limits(iphase).control.max(1,:) =  1;
limits(iphase).control.min(2,:) = -1;
limits(iphase).control.max(2,:) =  1;
limits(iphase).control.min(3,:) = -1;
limits(iphase).control.max(3,:) =  1;
limits(iphase).parameter.min    = [];
limits(iphase).parameter.max    = [];
limits(iphase).path.min    = 1;
limits(iphase).path.max    = 1;
guess(iphase).time = [t2; t3];
guess(iphase).state(:,1) = [rout(1); rout(1)];
guess(iphase).state(:,2) = [rout(2); rout(2)];
guess(iphase).state(:,3) = [rout(3); rout(3)];
guess(iphase).state(:,4) = [vout(1); vout(1)];
guess(iphase).state(:,5) = [vout(2); vout(2)];
guess(iphase).state(:,6) = [vout(3); vout(3)];
guess(iphase).state(:,7) = [m30; m3f];
guess(iphase).control(:,1) = [0; 0];
guess(iphase).control(:,2) = [0; 0];
guess(iphase).control(:,3) = [1; 1];
guess(iphase).parameter    = [];

iphase = 4;
limits(iphase).nodes = nodes;
limits(iphase).time.min = [t3 t3];
limits(iphase).time.max = [t3 t4];
limits(iphase).state.min(1,:) = [rmin rmin rmin];
limits(iphase).state.max(1,:) = [rmax rmax rmax];
limits(iphase).state.min(2,:) = [rmin rmin rmin];
limits(iphase).state.max(2,:) = [rmax rmax rmax];
limits(iphase).state.min(3,:) = [rmin rmin rmin];
limits(iphase).state.max(3,:) = [rmax rmax rmax];
limits(iphase).state.min(4,:) = [vmin vmin vmin];
limits(iphase).state.max(4,:) = [vmax vmax vmax];
limits(iphase).state.min(5,:) = [vmin vmin vmin];
limits(iphase).state.max(5,:) = [vmax vmax vmax];
limits(iphase).state.min(6,:) = [vmin vmin vmin];
limits(iphase).state.max(6,:) = [vmax vmax vmax];
limits(iphase).state.min(7,:) = [m4f m4f m4f];
limits(iphase).state.max(7,:) = [m40 m40 m40];
limits(iphase).control.min(1,:) = -1;
limits(iphase).control.max(1,:) =  1;
limits(iphase).control.min(2,:) = -1;
limits(iphase).control.max(2,:) =  1;
limits(iphase).control.min(3,:) = -1;
limits(iphase).control.max(3,:) =  1;
limits(iphase).parameter.min = [];
limits(iphase).parameter.max = [];
limits(iphase).path.min      = 1;
limits(iphase).path.max      = 1;
limits(iphase).event.min     = [af; ef; incf; Omf; omf];
limits(iphase).event.max     = [af; ef; incf; Omf; omf];
guess(iphase).time    = [t3; t4];
guess(iphase).state(:,1) = [rout(1) rout(1)];
guess(iphase).state(:,2) = [rout(2) rout(2)];
guess(iphase).state(:,3) = [rout(3) rout(3)];
guess(iphase).state(:,4) = [vout(1) vout(1)];
guess(iphase).state(:,5) = [vout(2) vout(2)];
guess(iphase).state(:,6) = [vout(3) vout(3)];
guess(iphase).state(:,7) = [m40; m4f];
guess(iphase).control(:,1) = [0; 0];
guess(iphase).control(:,2) = [0; 0];
guess(iphase).control(:,3) = [1; 1];
guess(iphase).parameter    = [];

ipair = 1; % First pair of phases to connect
linkages(ipair).left.phase = 1;
linkages(ipair).right.phase = 2;
linkages(ipair).min = [0; 0; 0; 0; 0; 0; -6*m_dry_srb];
linkages(ipair).max = [0; 0; 0; 0; 0; 0; -6*m_dry_srb];

ipair = 2; % Second pair of phases to connect
linkages(ipair).left.phase = 2;
linkages(ipair).right.phase = 3;
linkages(ipair).min = [0; 0; 0; 0; 0; 0; -3*m_dry_srb];
linkages(ipair).max = [0; 0; 0; 0; 0; 0; -3*m_dry_srb];

ipair = 3; % Third pair of phases to connect
linkages(ipair).left.phase = 3;
linkages(ipair).right.phase = 4;
linkages(ipair).min = [0; 0; 0; 0; 0; 0; -m_dry_first];
linkages(ipair).max = [0; 0; 0; 0; 0; 0; -m_dry_first];

setup.CONSTANTS = CONSTANTS;
setup.autoscale = 'on';
setup.name = 'Launch-Vehicle-Ascent';
setup.solver ='ipopt';
setup.method ='pseudospectral';
setup.funcs.cost = 'launchCost';
setup.funcs.dae = 'launchDae';
setup.funcs.event = 'launchEvent';
setup.funcs.link = 'launchConnect';
setup.derivatives = 'numerical';
setup.parallel    = 'no';
setup.limits = limits;
setup.linkages = linkages;
setup.guess = guess;

output = DMG(setup);
solution = output.solution;

