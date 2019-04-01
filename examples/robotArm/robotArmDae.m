function dae=robotArmDae(sol,setup);

CONSTANTS = setup.CONSTANTS;

t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;

Iphi   = ((CONSTANTS.L-x(:,1)).^3+x(:,1).^3)/3;
Itheta = Iphi.*sin(x(:,5)).^2;
x1dot = x(:,2);
x2dot = u(:,1)./CONSTANTS.L;
x3dot = x(:,4);
x4dot = u(:,2)./Itheta;
x5dot = x(:,6);
x6dot = u(:,3)./Iphi;

dae = [x1dot x2dot x3dot x4dot x5dot x6dot];
