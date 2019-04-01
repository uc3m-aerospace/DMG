function dae = goddardRocketDae(sol,setup)

CONSTANTS = setup.CONSTANTS;

t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;
h = x(:,1);
v = x(:,2);
m = x(:,3);
T = sol.control;
D = CONSTANTS.dragk.*(v.^2).*exp(-h/CONSTANTS.H);
hdot = v;
vdot = (T-D)./m-CONSTANTS.g0*ones(size(t));
% vdot = (T-CONSTANTS.sigma*(v.^2).*exp(-h./CONSTANTS.H))./m-CONSTANTS.g0*ones(size(t));
% mdot = -T./CONSTANTS.c;
mdot = -T./CONSTANTS.c;
if sol.phase==2,
    voverc = v/CONSTANTS.c;
    xmg = m*CONSTANTS.g0;
    term1 = (CONSTANTS.c^2).*(ones(size(t))+voverc)./(CONSTANTS.H*CONSTANTS.g0)-ones(size(t))-2./voverc;
    term2 = xmg./(ones(size(t))+4./voverc+2./(voverc.^2));
    %    term1 = T-CONSTANTS.sigma*(v.^2).*exp(-h/CONSTANTS.H)-m*CONSTANTS.g0;
    %    term2 = -m*CONSTANTS.g0./(ones(size(t))+4*CONSTANTS.c./v+2*CONSTANTS.c^2./v.^2).*((CONSTANTS.c^2/(CONSTANTS.H*CONSTANTS.g0))*(ones(size(t))+v./CONSTANTS.c)-ones(size(t))-2*CONSTANTS.c./v);
    %    path = term1+term2;
    path = T-D-xmg-term1.*term2;
    dae = [hdot vdot mdot path];
else
    dae = [hdot vdot mdot];
end;
