function dae = catalystMixingDae(sol,setup)

x1 = sol.state(:,1);
x2 = sol.state(:,2);
u  = sol.control;

x1dot = u.*(10*x2-x1);
x2dot = u.*(x1-10*x2)-(1-u).*x2;

dae = [x1dot x2dot];
