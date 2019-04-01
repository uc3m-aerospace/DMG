function [ri,vi] = launchoe2rv(oe,mu)
	a=oe(1); e=oe(2); i=oe(3); Om=oe(4); om=oe(5); nu=oe(6);
	p = a*(1-e*e);
	r = p/(1+e*cos(nu));
	rv = [r*cos(nu); r*sin(nu); 0];
	vv = sqrt(mu/p)*[-sin(nu); e+cos(nu); 0];
	cO = cos(Om);  sO = sin(Om);
	co = cos(om);  so = sin(om);
	ci = cos(i);   si = sin(i);
	R  = [cO*co-sO*so*ci  -cO*so-sO*co*ci  sO*si;
		  sO*co+cO*so*ci  -sO*so+cO*co*ci -cO*si;
		  so*si            co*si           ci];
	ri = R*rv;
	vi = R*vv;

