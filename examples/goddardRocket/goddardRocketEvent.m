function event = goddardRocketEvent(sol,setup);

CONSTANTS = setup.CONSTANTS;
t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
p  = sol.parameter;
iphase = sol.phase;

if iphase==2,
    h = xf(1);
    v = xf(2);
    m = xf(3);
    D = CONSTANTS.dragk.*(v.^2).*exp(-h/CONSTANTS.H);
    e0 = m*CONSTANTS.g0-(1+v/CONSTANTS.c).*D;
%     e0 = m*CONSTANTS.g0-(1+v/CONSTANTS.c)*CONSTANTS.sigma.*v.^2*exp(-h/CONSTANTS.H);
    event = e0;
else
    event = [];
end;
%
DV  = p(1);
angle = p(2);
%
Dv_r = DV*cos(angle);
Dv_o = DV*sin(angle);
v0_r = x0(3);
v0_o = x0(4);

theta_Target_sinodico = setup.theta_target_sinodico;
% calcular el theta en inercial
theta = xf(2);
theta_tierra = tf*tc;

event = [v0_r - Dv_r, v0_o - wE*1AU -Dv0,theta_Target_sinodico-theta-theta_tierra, ]











