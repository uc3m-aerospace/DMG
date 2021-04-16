function [dae] = InterplanetaryDae(sol,setup)
%
% Rename variables
%
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
alpha = sol.control(:,2);
beta  = sol.control(:,3);
%
r = sqrt( x.^2 + y.^2 + z.^2 );
%
% Thrust Acceleration
%
P0 = setup.P0;
P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r);
P = (2.6 *(P>2.6) + P.*(P<=2.6)).* (P>0.649);
TT = T.*(-1.9137 + 36.242 * P)/(setup.m0*setup.ac)/1000 .* (P>0.649);
%
ar = TT.*cos(alpha).*cos(beta)./m;
ao = TT.*sin(alpha).*cos(beta)./m;
%
theta = atan2(y,x);
ax = ar .* cos(theta) - ao .* sin(theta); 
ay = ar .* sin(theta) + ao .* cos(theta); 
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
mdot      = - T.*(0.47556 + 0.90209*P)*setup.tc/setup.m0/(1e6);
%
dae = [xdot ydot zdot vxdot vydot vzdot mdot];
%








