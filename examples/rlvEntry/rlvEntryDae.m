function dae=rlvEntryDae(sol,setup);

constants = setup.constants;

t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;

rad = x(:,1);
lon = x(:,2);
lat = x(:,3);
speed = x(:,4);
fpa = x(:,5);
azimuth = x(:,6);
aoa = u(:,1);
bank = u(:,2);

cd0 = constants.cd(1);
cd1 = constants.cd(2);
cd2 = constants.cd(3);
cl0 = constants.cl(1);
cl1 = constants.cl(2);
mu  = constants.mu;
rho0 = constants.rho0;
H = constants.H;
S = constants.S;
mass = constants.mass;

CD = cd0+cd1*aoa+cd2*aoa.^2;
altitude = rad-constants.Re;
rho = rho0*exp(-altitude/H);
CL = cl0+cl1*aoa;
gravity = mu./rad.^2;
dynamic_pressure = 0.5*rho.*speed.^2;
D = dynamic_pressure.*S.*CD./mass;
L = dynamic_pressure.*S.*CL./mass;
slon = sin(lon);
clon = cos(lon);
slat = sin(lat);
clat = cos(lat);
tlat = tan(lat);
sfpa = sin(fpa);
cfpa = cos(fpa);
sazi = sin(azimuth);
cazi = cos(azimuth);
cbank = cos(bank);
sbank = sin(bank);

raddot   = speed.*sfpa;
londot   = speed.*cfpa.*sazi./(rad.*clat);
latdot   = speed.*cfpa.*cazi./rad;
speeddot = -D-gravity.*sfpa;
fpadot   = (L.*cbank-cfpa.*(gravity-speed.^2./rad))./speed;
azidot   = (L.*sbank./cfpa + speed.^2.*cfpa.*sazi.*tlat./rad)./speed;

% No path constraints

dae = [raddot londot latdot speeddot fpadot azidot];
