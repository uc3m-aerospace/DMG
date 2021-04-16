function event = InterplanetaryEvent(sol,setup);
%
% Rename variables
%
flyby_planet     = setup.flyby_planet;
arrival_planet   = setup.arrival_planet;
departure_planet = setup.departure_planet; 
%
Initial_Date     = setup.Initial_Date;
Final_Date       = setup.Final_Date;
%
tc               = setup.tc;
lc               = setup.lc;
vc               = setup.vc;
%
t0     = sol.initial.time;
xx0    = sol.initial.state;
tf     = sol.terminal.time;
xxf    = sol.terminal.state;
p      = sol.parameter;
iphase = sol.phase;
% RIGHT PHASE STATE
xf   = xxf(1);
yf   = xxf(2);
zf   = xxf(3);
vxf  = xxf(4);
vyf  = xxf(5);
vzf  = xxf(6);
% LEFT PHASE STATE
x0   = xx0(1);
y0   = xx0(2);
z0   = xx0(3);
vx0  = xx0(4);
vy0  = xx0(5);
vz0  = xx0(6);
%
et0_min    = cspice_str2et(Initial_Date);
et0_max    = cspice_str2et(Final_Date);
et0_factor = p(end);
et0        = et0_min + (et0_max - et0_min)*et0_factor; 
%
% Flyby constraints
%
if iphase==1
    %
    alpha1 = p(3);
    alpha2 = p(4);
    v_inf0 = p(end-1);
    %
    S0  = cspice_spkezr(departure_planet, et0, 'ECLIPJ2000', 'NONE', 'SUN');
    rp0 = S0(1:3)*1e3/lc;
    vp0 = S0(4:6)*1e3/vc + v_inf0*1e3/vc*[cos(alpha1)*cos(alpha2),cos(alpha1)*sin(alpha2),sin(alpha1)]';
    %vp0 = S0(4:6)*1e3/vc + v_inf0*1e3/vc*[cos(alpha2),sin(alpha2),0]';
    %vp0 = S0(4:6)*1e3/vc + v_inf0*1e3/vc*S0(4:6)/norm(S0(4:6));
    et  = et0 + tf*tc;
    
    S1  = cspice_spkezr(flyby_planet(iphase), et, 'ECLIPJ2000', 'NONE', 'SUN');
    rpf = S1(1:3)*1e3/lc;
    
    event = [x0-rp0(1);y0-rp0(2);(z0-rp0(3));vx0-vp0(1);vy0-vp0(2);(vz0-vp0(3));xf-rpf(1);yf-rpf(2);(zf-rpf(3));tf-t0];
  
elseif iphase == setup.n_phases
    
    et  = et0 + tf*tc;
        
    S1  = cspice_spkezr(arrival_planet, et , 'ECLIPJ2000', 'NONE', 'SUN');
    rpf = S1(1:3)*1e3/lc;
    vpf = S1(4:6)*1e3/vc;

    event = [xf-rpf(1);yf-rpf(2);(zf-rpf(3));tf-t0;(vpf(1)-vxf); (vpf(2)-vyf);(vpf(3)-vzf)];
    
else
    
    et  = et0 + tf*tc;
        
    S1  = cspice_spkezr(flyby_planet(iphase), et, 'ECLIPJ2000', 'NONE', 'SUN');
    rpf = S1(1:3)*1e3/lc;


    event = [xf-rpf(1);yf-rpf(2);(zf-rpf(3));tf-t0];
    
end






