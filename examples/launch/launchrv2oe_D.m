function Doe = launchrv2oe_D(rv,vv,mu,Re);

rbar = Re;
vbar = sqrt(mu/rbar);

Doe = zeros(5,6);

xfe = rv(1)/rbar;
yfe = rv(2)/rbar;
zfe = rv(3)/rbar;
xdfe = vv(1)/vbar;
ydfe = vv(2)/vbar;
zdfe = vv(3)/vbar;

rfinal = sqrt(xfe^2+yfe^2+zfe^2);
vfinal = sqrt(xdfe^2+ydfe^2+zdfe^2);
rvec = [xfe; yfe; zfe];
vvec = [xdfe; ydfe; zdfe];

evec = ((vfinal^2 - 1/rfinal)*rvec - dot(rvec,vvec)*vvec);
ef = norm(evec);

E = vfinal^2/2 - 1/rfinal;
af = -1/2/E;

h = cross(rvec,vvec);
incf = acos(dot(h,[0;0;1])/norm(h));

Doe(1,1) = (1/(rfinal*vfinal^2-2) - rfinal*vfinal^2/(rfinal*vfinal^2-2)^2)*xfe/rfinal;             %xfe
Doe(1,2) = (1/(rfinal*vfinal^2-2) - rfinal*vfinal^2/(rfinal*vfinal^2-2)^2)*yfe/rfinal;             %yfe
Doe(1,3) = (1/(rfinal*vfinal^2-2) - rfinal*vfinal^2/(rfinal*vfinal^2-2)^2)*zfe/rfinal;             %zfe
Doe(1,4) = -(2*rfinal^2/(rfinal*vfinal^2-2)^2)*xdfe;             %xdfe
Doe(1,5) = -(2*rfinal^2/(rfinal*vfinal^2-2)^2)*ydfe;            %ydfe
Doe(1,6) = -(2*rfinal^2/(rfinal*vfinal^2-2)^2)*zdfe;             %zdfe

e1 = evec(1);
e2 = evec(2);
e3 = evec(3);

de1_dx = vfinal^2 - 1/rfinal + xfe^2/rfinal^3 - xdfe^2;
de1_dy = xfe*yfe/rfinal^3 - ydfe*xdfe;
de1_dz = xfe*zfe/rfinal^3 - zdfe*xdfe;
de1_dxd = -yfe*ydfe - zfe*zdfe;
de1_dyd = 2*xfe*ydfe - yfe*xdfe;
de1_dzd = 2*xfe*zdfe - zfe*xdfe;

de2_dx = xfe*yfe/rfinal^3 - ydfe*xdfe;
de2_dy = vfinal^2 - 1/rfinal + yfe^2/rfinal^3 - ydfe^2;
de2_dz = yfe*zfe/rfinal^3 - ydfe*zdfe;
de2_dxd = 2*yfe*xdfe - xfe*ydfe;
de2_dyd = -xfe*xdfe - zfe*zdfe;
de2_dzd = 2*yfe*zdfe - zfe*ydfe;

de3_dx = xfe*zfe/rfinal^3 - zdfe*xdfe;
de3_dy = yfe*zfe/rfinal^3 - ydfe*zdfe;
de3_dz = vfinal^2 - 1/rfinal + zfe^2/rfinal^3 - zdfe^2;
de3_dxd = 2*zfe*xdfe - xfe*zdfe;
de3_dyd = 2*zfe*ydfe - yfe*zdfe;
de3_dzd = -xfe*xdfe - yfe*ydfe;

Doe(2,1) =  -1/ef*(e1*de1_dx + e2*de2_dx + e3*de3_dx);          %xfe
Doe(2,2) =  -1/ef*(e1*de1_dy + e2*de2_dy + e3*de3_dy);          %yfe
Doe(2,3) =  -1/ef*(e1*de1_dz + e2*de2_dz + e3*de3_dz);           %zfe
Doe(2,4) =  -1/ef*(e1*de1_dxd + e2*de2_dxd + e3*de3_dxd);        %xdfe
Doe(2,5) =  -1/ef*(e1*de1_dyd + e2*de2_dyd + e3*de3_dyd);       %ydfe
Doe(2,6) =  -1/ef*(e1*de1_dzd + e2*de2_dzd + e3*de3_dzd);        %zdfe

%dhi_dx = 0;
dhi_dy = zdfe;
dhi_dz = -ydfe;
%dhi_dxd = 0;
dhi_dyd = -zfe;
dhi_dzd = yfe;

dhj_dx = -zdfe;
%dhj_dy = 0;
dhj_dz = xdfe;
dhj_dxd = zfe;
%dhj_dyd = 0;
dhj_dzd = -xfe;

dhk_dx = ydfe;
dhk_dy = -xdfe;
%dhk_dz = 0;
dhk_dxd = -yfe;
dhk_dyd = xfe;
%dhk_dzd = 0;

hi = h(1);
hj = h(2);
hk = h(3);
hm = norm(h);

Doe(3,1) =  1/sqrt(1-(hk/hm)^2)*(dhk_dx/hm - hk*(hj*dhj_dx + hk*dhk_dx)/hm^3);          %xfe
Doe(3,2) =  1/sqrt(1-(hk/hm)^2)*(dhk_dy/hm - hk*(hi*dhi_dy + hk*dhk_dy)/hm^3);          %yfe
Doe(3,3) =  1/sqrt(1-(hk/hm)^2)*(- hk*(hi*dhi_dz + hj*dhj_dz)/hm^3);           %zfe
Doe(3,4) =  1/sqrt(1-(hk/hm)^2)*(dhk_dxd/hm - hk*(hj*dhj_dxd + hk*dhk_dxd)/hm^3);        %xdfe
Doe(3,5) =  1/sqrt(1-(hk/hm)^2)*(dhk_dyd/hm - hk*(hi*dhi_dyd + hk*dhk_dyd)/hm^3);       %ydfe
Doe(3,6) =  1/sqrt(1-(hk/hm)^2)*(- hk*(hi*dhi_dzd + hj*dhj_dzd)/hm^3);        %zdfe

% Omega   % xfe
Doe(4,1) = -(zdfe/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)-1/2*(-zfe*xdfe+xfe*zdfe)/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)*(-2*zfe*xdfe*zdfe+2*xfe*zdfe^2))/(1-(-zfe*xdfe+xfe*zdfe)^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2))^(1/2);
Doe(4,2) = 1/2*(-zfe*xdfe+xfe*zdfe)/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)*(2*yfe*zdfe^2-2*ydfe*zfe*zdfe)/(1-(-zfe*xdfe+xfe*zdfe)^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2))^(1/2);
Doe(4,3) = -(-xdfe/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)-1/2*(-zfe*xdfe+xfe*zdfe)/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)*(2*zfe*xdfe^2-2*xdfe*xfe*zdfe-2*yfe*zdfe*ydfe+2*zfe*ydfe^2))/(1-(-zfe*xdfe+xfe*zdfe)^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2))^(1/2);
Doe(4,4) = -(-zfe/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)-1/2*(-zfe*xdfe+xfe*zdfe)/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)*(2*zfe^2*xdfe-2*zdfe*xfe*zfe))/(1-(-zfe*xdfe+xfe*zdfe)^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2))^(1/2);
Doe(4,5) = 1/2*(-zfe*xdfe+xfe*zdfe)/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)*(-2*yfe*zdfe*zfe+2*zfe^2*ydfe)/(1-(-zfe*xdfe+xfe*zdfe)^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2))^(1/2);
Doe(4,6) = -(xfe/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)-1/2*(-zfe*xdfe+xfe*zdfe)/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)*(-2*zfe*xdfe*xfe+2*xfe^2*zdfe+2*yfe^2*zdfe-2*yfe*zfe*ydfe))/(1-(-zfe*xdfe+xfe*zdfe)^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2))^(1/2);

% omega
Doe(5,1) = -((zdfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(-zfe*xdfe+xfe*zdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))+(yfe*zdfe-zfe*ydfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*yfe-ydfe*xdfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)*(-2*zfe*xdfe*zdfe+2*xfe*zdfe^2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(3/2)*(2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*yfe-ydfe*xdfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*zfe-zdfe*xdfe)))/(1-((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2))^(1/2);
Doe(5,2) = -(((-zfe*xdfe+xfe*zdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*yfe-ydfe*xdfe)+zdfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)+(yfe*zdfe-zfe*ydfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*yfe^2+xdfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2)))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)*(2*yfe*zdfe^2-2*zfe*zdfe*ydfe)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(3/2)*(2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*yfe-ydfe*xdfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*yfe^2+xdfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*yfe*zfe-zdfe*ydfe)))/(1-((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2))^(1/2);
Doe(5,3) = -((-xdfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(-zfe*xdfe+xfe*zdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*zfe-zdfe*xdfe)-ydfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)+(yfe*zdfe-zfe*ydfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*yfe*zfe-zdfe*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)*(2*xdfe^2*zfe-2*xdfe*xfe*zdfe-2*zdfe*ydfe*yfe+2*zfe*ydfe^2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(3/2)*(2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*xfe*zfe-zdfe*xdfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*yfe*zfe-zdfe*ydfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)*(1/(xfe^2+yfe^2+zfe^2)^(3/2)*zfe^2+xdfe^2+ydfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))))/(1-((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2))^(1/2);
Doe(5,4) = -((-zfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(-zfe*xdfe+xfe*zdfe)*(-yfe*ydfe-zdfe*zfe)+(yfe*zdfe-zfe*ydfe)*(2*yfe*xdfe-xfe*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)*(2*zfe^2*xdfe-2*zdfe*xfe*zfe)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(3/2)*(2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)*(-yfe*ydfe-zdfe*zfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)*(2*yfe*xdfe-xfe*ydfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)*(2*zfe*xdfe-xfe*zdfe)))/(1-((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2))^(1/2);
Doe(5,5) = -(((-zfe*xdfe+xfe*zdfe)*(2*xfe*ydfe-yfe*xdfe)-zfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)+(yfe*zdfe-zfe*ydfe)*(-xdfe*xfe-zdfe*zfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)*(-2*zfe*zdfe*yfe+2*ydfe*zfe^2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(3/2)*(2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)*(2*xfe*ydfe-yfe*xdfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)*(-xdfe*xfe-zdfe*zfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)*(2*zfe*ydfe-yfe*zdfe)))/(1-((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2))^(1/2);
Doe(5,6) = -((xfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(-zfe*xdfe+xfe*zdfe)*(2*xfe*zdfe-zfe*xdfe)+yfe*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)+(yfe*zdfe-zfe*ydfe)*(2*yfe*zdfe-zfe*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(3/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(1/2)*(-2*zfe*xdfe*xfe+2*xfe^2*zdfe+2*yfe^2*zdfe-2*yfe*zfe*ydfe)-1/2*((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)^(1/2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2)^(3/2)*(2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)*(2*xfe*zdfe-zfe*xdfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)*(2*yfe*zdfe-zfe*ydfe)+2*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)*(-xdfe*xfe-yfe*ydfe)))/(1-((-zfe*xdfe+xfe*zdfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)+(yfe*zdfe-zfe*ydfe)*((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe))^2/(zfe^2*xdfe^2-2*zfe*xdfe*xfe*zdfe+xfe^2*zdfe^2+yfe^2*zdfe^2-2*yfe*zdfe*zfe*ydfe+zfe^2*ydfe^2)/(((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*xfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*xdfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*yfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*ydfe)^2+((xdfe^2+ydfe^2+zdfe^2-1/(xfe^2+yfe^2+zfe^2)^(1/2))*zfe-(xdfe*xfe+yfe*ydfe+zdfe*zfe)*zdfe)^2))^(1/2);

Doe(1,:) = Doe(1,:)*rbar;
Doe(:,1:3) = Doe(:,1:3)/rbar;
Doe(:,4:6) = Doe(:,4:6)/vbar;
% fix quadrants
Doe(1:3,:) = -Doe(1:3,:);
K  = [0;0;1];
hv = cross(rv,vv);
nv = cross(K,hv);
if ( nv(2) < 0-eps )
    Doe(4,:) = -Doe(4,:);
end
if e3 < 0
    Doe(5,:) = -Doe(5,:);
end


