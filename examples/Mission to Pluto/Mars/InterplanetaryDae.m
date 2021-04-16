function [dae] = InterplanetaryDae(sol,setup)
%
% Rename variables
%
t     = sol.time;
xx    = sol.state;
x     = xx(:,1);
y     = xx(:,2);
z     = xx(:,3);
vx    = xx(:,4);
vy    = xx(:,5);
vz    = xx(:,6);
m     = xx(:,7);
%
T     = sol.control(:,1);
%T     = atan(1000.*T)*2/pi;

switch_cons = T.*(1-T);
alpha = sol.control(:,2);
beta  = sol.control(:,3);
%
launcher = setup.launcher;
%
c3 = sol.parameter(end-1)^2;
%
m0 = launcher_model(launcher,c3);
%
r  = sqrt( x.^2 + y.^2 + z.^2 );
%
% Thrust Acceleration
%
P0 = setup.P0;
Isp = sol.control(:,4)*1000;
%--------------------------------------------------------------------------
%
% MISSION DAWN (ENGINE NSTAR)
%
%P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r);
%
% Thrust
%
%TT    =   T.*(-1.9137 + 36.242 * P)/(setup.m0*setup.ac)/1000 .* (P>0);
%
% Propellant rate
%
%mdot  = - T.*(0.47556 + 0.90209*P).* (P>=0.649)*setup.tc/setup.m0/(1e6);
%--------------------------------------------------------------------------
%
% MISSION CAESAR (ENGINE NEXT)
%
% Power subsystem
%
%P   = (P0./ r.^2-1)*0.85/2;
%P   = P.*(P>0);
%Pa  = 7.36;
%P   =  Pa*(P>Pa) + P.*(P<=Pa) ;
%
% Thrust
%
%TT  =  2*T.*(1.19388817E-02 + 1.60989424E-02*P + 1.14181412E-02*P.^2 -2.04053417E-03*P.^3 + 1.01855017E-04*P.^4)/(m0*setup.ac).* (P>0.64);
%TT  = TT.*(TT>0);
%
% Propellant rate
%
%mdot  = - 2*T.*(2.75956482E-06 - 1.71102132E-06*P + 1.21670237E-06*P.^2  -2.07253445E-07*P.^3 + 1.10213671E-08*P.^4).* (TT>0)*setup.tc/m0.*(P>0.64);
%
%--------------------------------------------------------------------------
% MISSION CAESAR (ENGINE NEXT-C)
%
% Power subsystem
%
P   = P0;
P_margin = 0.15;
eta = 0.6;
duty_cycle = 0.9;
decay = 0.02; % percentage per year
decay_factor = (1-decay).^(t*setup.tc/(365*24*3600));
%
% Thrust
%
TT  =  duty_cycle*T.*(2*eta*(1-P_margin)*(decay_factor*1000-200))./(Isp*setup.g0) / (m0*setup.ac).* (P>0);
TT  = TT.*(TT>0);
%
% Propellant rate
%
mdot  = - T.*(TT./(Isp*setup.g0))*setup.tc*setup.ac*(P>0);
%
%-------------------------------------------------------------------------
ax = TT.*cos(alpha).*cos(beta)./m;
ay = TT.*sin(alpha).*cos(beta)./m;
az = TT .* sin(beta)./m;
%
% Dynamical system
%
xdot  = vx;
ydot  = vy;
zdot  = vz;
vxdot = -1./r.^3 .*x + ax;
vydot = -1./r.^3 .*y + ay;
vzdot = -1./r.^3 .*z + az;
%
dae = [xdot ydot zdot vxdot vydot vzdot mdot r switch_cons];
%








