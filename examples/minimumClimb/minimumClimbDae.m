function daeout = minimumClimbDae(soldae,setup);

CONSTANTS = setup.CONSTANTS;

CoF = CONSTANTS.CoF;
CoFZ = CONSTANTS.CoFZ;

t = soldae.time;
x = soldae.state;
u = soldae.control;
p = soldae.parameter;
h = x(:,1);
E = x(:,2);
fpa = x(:,3);

g = 9.80665;
m = 37000./2.2;
S = 60;
hbar = h./1000;

%rho calculation
z = CoFZ(1).*hbar+CoFZ(2).*hbar.^2+CoFZ(3).*hbar.^3+CoFZ(4).*hbar.^4;
r = 1.0228066.*exp(-z);
y = -0.12122693.*hbar+r-1.0228055;
rho = 1.225.*exp(y);
%rho calculation

%speed of sound%
theta = 292.1-8.87743.*hbar+0.193315.*hbar.^2+(3.72e-3).*hbar.^3;
a     = 20.0468.*sqrt(theta);
%speed of sound%

%Velocity and mach
Eminush = (E-h).^2;
Eminush = sqrt(Eminush);
v = sqrt(2.*g.*Eminush);

M = v./a;
%Velocity and mach

%Who are lift and drag
q = 0.5.*rho.*v.*v.*S;
L = m.*g.*u;
M0 = M.^0;
M1 = M.^1;
M2 = M.^2;
M3 = M.^3;
M4 = M.^4;
M5 = M.^5;
numeratorCD0 = CoF(1,1).*M0+CoF(1,2).*M1+CoF(1,3).*M2+CoF(1,4).*M3+CoF(1,5).*M4;
denominatorCD0 = CoF(2,1).*M0+CoF(2,2).*M1+CoF(2,3).*M2+CoF(2,4).*M3+CoF(2,5).*M4;
Cd0 = numeratorCD0./denominatorCD0;
numeratorK = CoF(3,1).*M0+CoF(3,2).*M1+CoF(3,3).*M2+CoF(3,4).*M3+CoF(3,5).*M4;
denominatorK = CoF(4,1).*M0+CoF(4,2).*M1+CoF(4,3).*M2+CoF(4,4).*M3+CoF(4,5).*M4+CoF(4,6).*M5;
K   = numeratorK./denominatorK;
D = q.*(Cd0+K.*((m.^2).*(g.^2)./(q.^2)).*(u.^2));
%Who are lift and drag

%Who is thrust
e0 = CoF(5,1).*M0+CoF(6,1).*M1+CoF(7,1).*M2+CoF(8,1).*M3+CoF(9,1).*M4+CoF(10,1).*M5;
e1 = CoF(5,2).*M0+CoF(6,2).*M1+CoF(7,2).*M2+CoF(8,2).*M3+CoF(9,2).*M4+CoF(10,2).*M5;
e2 = CoF(5,3).*M0+CoF(6,3).*M1+CoF(7,3).*M2+CoF(8,3).*M3+CoF(9,3).*M4+CoF(10,3).*M5;
e3 = CoF(5,4).*M0+CoF(6,4).*M1+CoF(7,4).*M2+CoF(8,4).*M3+CoF(9,4).*M4+CoF(10,4).*M5;
e4 = CoF(5,5).*M0+CoF(6,5).*M1+CoF(7,5).*M2+CoF(8,5).*M3+CoF(9,5).*M4+CoF(10,5).*M5;
e5 = CoF(5,6).*M0+CoF(6,6).*M1+CoF(7,6).*M2+CoF(8,6).*M3+CoF(9,6).*M4+CoF(10,6).*M5;

T = (e0.*hbar.^0+e1.*hbar.^1+e2.*hbar.^2+e3.*hbar.^3+e4.*hbar.^4+e5.*hbar.^5).*9.80665./2.2;
%Who is thrust

%************************************************

%User Input - Give me f in dot(x) = f
%*********************

Edot = v./(m.*g).*(T-D);
hdot = v.*sin(fpa);
fpadot = g./v.*(L./(m.*g)-cos(fpa));


daeout = [hdot Edot fpadot];

