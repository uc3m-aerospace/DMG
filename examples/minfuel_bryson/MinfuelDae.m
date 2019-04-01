function [dae,Ddae] = MinfuelDae(sol,setup)

global constants

t = sol.time;
x = sol.state;
r = x(:,1);
u = x(:,2);
v = x(:,3);
con = sol.control;
u1=con(:,1);
u2=con(:,2);
p2 = sol.parameter;
p1 = constants.p1;



rdot = u ;
udot = v.^2 ./ r - 1 ./(r.^2) + u1 ./ (p1-p2.*t);
vdot = -u.*v./r + u2./(p1-p2.*t);
path = u1.^2+u2.^2;
dae = [rdot udot vdot path];

%%if nargout == 2
    %xdotx = zeros(size(t));
    %xdoty = zeros(size(t));
    %xdotv = u(:,1);
    %xdotu1 = x(:,3);
    %xdotu2 = zeros(size(t));
    %xdott = zeros(size(t));

    %ydotx = zeros(size(t));
    %ydoty = zeros(size(t));
    %ydotv = u(:,2);
    %ydotu1 = zeros(size(t));
    %ydotu2 = x(:,3);
    %ydott = zeros(size(t));

    %vdotx = zeros(size(t));
    %vdoty = zeros(size(t));
    %vdotv = zeros(size(t));
    %vdotu1 = zeros(size(t));
    %vdotu2 = -constants.gravity.*ones(size(t));
    %vdott = zeros(size(t));

    %pathx = zeros(size(t));
    %pathy = zeros(size(t));
    %pathv = zeros(size(t));
    %pathu1  = 2*u(:,1);
    %pathu2  = 2*u(:,2);
    %patht = zeros(size(t));
    %Ddae = [xdotx xdoty xdotv xdotu1 xdotu2 xdott;
          %  ydotx ydoty ydotv ydotu1 ydotu2 ydott;
          %  vdotx vdoty vdotv vdotu1 vdotu2 vdott;
           % pathx pathy pathv pathu1 pathu2 patht];
%end