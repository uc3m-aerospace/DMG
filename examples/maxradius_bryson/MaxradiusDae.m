function [dae] = MaxradiusDae(sol,setup)

t = sol.time;
x = sol.state;
r = x(:,1);
u = x(:,2);
v = x(:,3);
con = sol.control;
u1=con(:,1);
u2=con(:,2);
p1 = setup.p1;
p2 = setup.p2;

tf = 3.32;

t= t*tf;

rdot = u ;
udot = v.^2 ./ r - 1 ./(r.^2) + u1./ (p1-p2.*t);
vdot = -u.*v./r + u2./(p1-p2.*t);

path = u1.^2+u2.^2-1;

dae = [tf*rdot tf*udot tf*vdot path];


