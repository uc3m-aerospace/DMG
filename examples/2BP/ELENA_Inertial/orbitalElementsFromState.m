%% Orbital Elements from State Vector
function coe = orbitalElementsFromState(x0)
%% This function is based on Algortihm 4.2 from the book 
%% Orbital Mechanics for Engineering Students by H.D. Curtis, 
%% Third Edition (2014). 
%% The algorithm has been generalised to accept multiple objects.

%% Set Globals 
mu    = 1.26686534e17;           % km3 s−2

%% Numnber of objects
sz = size(x0);
num = sz(2);

%% Separate position and velocity vectors. 
R = x0(1:3,:);
V = x0(4:6,:);

%% Module of R and V
r = vecnorm(R);
v = vecnorm(V);
%% Radial velocity
vr = dot(R,V)./r;
%% Angular momentum
H  = cross(R,V);
h  = vecnorm(H);
kk = H./h;
%% Equation 4.7: orbit's inclination
incl = acos(H(3,:)./h);

%% Equation 4.8: Node line
KN = [0; 0; 1]*ones(1,num);
N = cross(KN,H);
n = vecnorm(N);

%% Equation 4.9: Ascending Node
%% RA = 0 if n = 0
cond1 = (n ~= 0);
if cond1 == 0
RA = 0;
N  = [1;0;0];
n  = vecnorm(N);
else
RA = acos(N(1,:)./n);
end
    
%% Modify only those objects with N(2) < 0
cond2 = (N(2,:) < 0);
RA(cond2) = 2*pi - RA(cond2);

%% Equation 4.10: Eccentricity
E = 1/mu*((v.^2 - mu./r).*R - r.*vr.*V);
e = vecnorm(E);

%% Equation 4.12: Argument of perigee
%% w = 0 if n = 0 or e < eps
eps = 1.e-12;
cond3 = (n ~= 0 & e > eps);
w = acos(dot(N,E)./n./e)*cond3;
%% Modify only those objects with
cond4 = (n ~= 0 & e > eps & dot(E,cross(kk,N)) < 0);
w(cond4) = 2*pi - w(cond4);
w(w == 2*pi) = 0;

%% Equation 4.13a (incorporating the case e = 0): True anomaly
cond5 = (e > eps);
TA = acos(dot(E,R)./e./r).*cond5;
cond6 = (e > eps & vr < 0);
TA(cond6) = 2*pi - TA(cond6);
%% Modify only those objects with e < eps
cp = cross(N,R);
cond7 = (cp(3,:) < 0);
if any(~cond5)
    TA(~cond5) = acos(dot(N(:,~cond5),R(:,~cond5))./n(~cond5)./r(~cond5));
    TA(~cond5 & cond7) = 2*pi - TA(~cond5 & cond7);
end

%% Equation 4.62 (a < 0 for a hyperbola): Semimajor axis
a = (h.^2/mu)./(1 - e.^2);

%% Output
coe = [h; e; RA; incl; w; TA; a];

end