function [dae,Ddae] = brysonMaxrangeDae(sol,setup);

global constants

t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;

xdot = x(:,3).*u(:,1);
ydot = x(:,3).*u(:,2);
vdot = constants.acc-constants.gravity.*u(:,2);
path = u(:,1).^2+u(:,2).^2;
dae = [xdot ydot vdot path];

if nargout == 2
    xdotx = zeros(size(t));
    xdoty = zeros(size(t));
    xdotv = u(:,1);
    xdotu1 = x(:,3);
    xdotu2 = zeros(size(t));
    xdott = zeros(size(t));

    ydotx = zeros(size(t));
    ydoty = zeros(size(t));
    ydotv = u(:,2);
    ydotu1 = zeros(size(t));
    ydotu2 = x(:,3);
    ydott = zeros(size(t));

    vdotx = zeros(size(t));
    vdoty = zeros(size(t));
    vdotv = zeros(size(t));
    vdotu1 = zeros(size(t));
    vdotu2 = -constants.gravity.*ones(size(t));
    vdott = zeros(size(t));

    pathx = zeros(size(t));
    pathy = zeros(size(t));
    pathv = zeros(size(t));
    pathu1  = 2*u(:,1);
    pathu2  = 2*u(:,2);
    patht = zeros(size(t));
    Ddae = [xdotx xdoty xdotv xdotu1 xdotu2 xdott;
            ydotx ydoty ydotv ydotu1 ydotu2 ydott;
            vdotx vdoty vdotv vdotu1 vdotu2 vdott;
            pathx pathy pathv pathu1 pathu2 patht];
end