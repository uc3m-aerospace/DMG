function dae = linearTangentDae(sol,setup);

a = 100;

t = sol.time;
x = sol.state;
u = sol.control;
x1dot = x(:,3);
x2dot = x(:,4);
x3dot = a.*cos(u);
x4dot = a.*sin(u);

dae = [x1dot x2dot x3dot x4dot];

