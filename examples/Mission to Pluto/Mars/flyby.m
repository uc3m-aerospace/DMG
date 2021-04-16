function [vf] = flyby(v,vp,mu_cb,fb_b,hpm,cc,lc)
%--------------------------------------------------------------------------
% PERFORMED FLYBY IN MARS
%--------------------------------------------------------------------------
%
%id      = cspice_bodn2c(fb_b);

id      = str2double(fb_b);
id      = id*100+99;

R_flyby = mean( cspice_bodvcd(id,'RADII',3) )*1e3/lc;
mum     = cspice_bodvcd(id,'GM',3)*1e9/mu_cb;

%--------------------------------------------------------------------------
% project velocity onto the inertial frame
%--------------------------------------------------------------------------
%
v_inf1       = v - vp;
v_inf        = norm(v_inf1);
% rotation axis
sb  = v_inf1/norm(v_inf1);
tb  = cross(sb,[0;0;1])/norm(cross(sb,[0;0;1]));
% Obtain orientation of the orbital plane
npi = tb*cos(-(pi/2-cc)) + cross(sb,tb)*sin(-(pi/2-cc)) + sb*(dot(sb,tb))*(1-cos(-(pi/2-cc)));
% In-plane rotation angle
delta  =  2*asin( 1 ./(  1 + v_inf^2 *(R_flyby + hpm)/mum  ) ) ;
%
vf_inf2  = v_inf1*cos(delta) + cross(npi,v_inf1)*sin(delta) + npi*(dot(npi,v_inf1))*(1-cos(delta));
%
vf = vf_inf2 + vp;



