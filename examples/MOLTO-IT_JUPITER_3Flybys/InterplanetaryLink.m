function link = InterplanetaryLink(sol,setup);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flyby_planeta   = setup.flyby_planet;
Initial_Date     = setup.Initial_Date;
Final_Date       = setup.Final_Date;
lc             = setup.lc;
tc             = setup.tc;
vc             = setup.vc;
mu_cb          = setup.mu;
%hpm            = setup.hpm;
%cc             = 0; 
et0            = setup.et0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xf_left = sol.left.state;
tf      = sol.left.time;
t0      = sol.right.time;
p_left = sol.left.parameter;
hpm = p_left(1)*1e3/setup.lc*1000;
cc  = p_left(2);
%
phase    = sol.left.phase;
%
flyby_planet = flyby_planeta(phase);
%flyby_planet = 'Jupiter';
%
x0_right = sol.right.state;
p_right = sol.right.parameter;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LEFT PHASE STATE
vx0 = xf_left(4);
vy0 = xf_left(5);
vz0 = xf_left(6);
m0  = xf_left(7);
% RIGHT PHASE STATE
vxf = x0_right(4);
vyf = x0_right(5);
vzf = x0_right(6);
mf  = x0_right(7);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
et0_min    = cspice_str2et(Initial_Date);
et0_max    = cspice_str2et(Final_Date);
et0_factor = p_left(end);
et0        = et0_min + (et0_max - et0_min)*et0_factor; 
%
et = et0 + tf*tc;
%
v0  = [vx0;vy0;vz0];
%
S1  = cspice_spkezr(flyby_planet, et, 'ECLIPJ2000', 'NONE', 'SUN');
%
vfb = S1(4:6)*1e3/vc;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

[vf_out] = flyby(v0,vfb,mu_cb,flyby_planet,hpm,cc,lc);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

link = [xf_left(1:3)-x0_right(1:3); vxf-vf_out(1); vyf-vf_out(2);vzf-vf_out(3); tf-t0 ; m0-mf; p_left(end)-p_right(end) ]*1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

