function oe = launchrv2oe(rv,vv,mu);

K  = [0;0;1];
hv = cross(rv,vv);
nv = cross(K,hv);
n  = sqrt(nv.'*nv);
h2 = (hv.'*hv);
v2 = (vv.'*vv);
r  = sqrt(rv.'*rv);
ev = 1/mu *( (v2-mu/r)*rv - (rv.'*vv)*vv );
p  = h2/mu;
%
% now compute the oe's
%
e  = sqrt(ev.'*ev);		% eccentricity
a  = p/(1-e*e);			% semimajor axis
i  = acos(hv(3)/sqrt(h2));	% inclination
Om = acos(nv(1)/n);		% RAAN
if ( nv(2) < 0-eps )		% fix quadrant
    Om = 2*pi-Om;
end;
om = acos(nv.'*ev/n/e);		% arg of periapsis
if ( ev(3) < 0 )		% fix quadrant
    om = 2*pi-om;
end;
nu = acos(ev.'*rv/e/r);		% true anomaly
if ( rv.'*vv < 0 )		% fix quadrant
    nu = 2*pi-nu;
end;
oe = [a; e; i; Om; om; nu];		% assemble "vector"

