function get_results
%
EM  = load_spice_kernels('/Users/davidmorante/Desktop/MicePackage/');
%
n_fb = 1;
%
lc  = 149597870.700e03;
mu  = 132712440018e09;
tc  = sqrt(lc^3/mu);
vc  = lc/tc;
ac  = lc/tc^2;
%
fid = fopen( ['OCP_SUMMARY.txt'], 'wt' );
%
for i = 1
%   
load('Earth-Mars-Pluto.mat')
%
fb_b =[setup.flyby_planet,setup.arrival_planet];
%
subpath = ['./solution'];
%
if not ( isdir(subpath) )
   mkdir(subpath)
end
%
x = [ solution(1).state(:,1);solution(2).state(:,1)];
y = [ solution(1).state(:,2);solution(2).state(:,2)];
z = [ solution(1).state(:,3);solution(2).state(:,3)];
%
fig = figure;
plot3(x,y,z)
%
axis equal
grid on
title(['Trajetory'],'Interpreter','latex','Fontsize',14)
xlabel('X (AU)','Interpreter','latex','Fontsize',14)
ylabel('Y (AU)','Interpreter','latex','Fontsize',14)
zlabel('Z (AU)','Interpreter','latex','Fontsize',14)
saveas(fig,[subpath,'/Trajectory'],'epsc')
saveas(fig,[subpath,'/Trajectory'],'fig')
close(fig)
%
% Obtener fecha de lanzamiento
%
t0 = solution(1).parameter(end);
%
% Obtener tiempos de vuelo
%
t = zeros(1,n_fb+1);
for j = 1:n_fb+1
t(j) = (solution(j).time(end)-solution(j).time(1))*tc/(3600*24);
end
%
% Obtener Altura de Flybys
%
rfb = zeros(1,n_fb);
for j = 1:n_fb
rfb(j) = (solution(j).parameter(1))*1e3;
   if rfb(j) < 200
    rfb(j) = 200;
   end
end
%
% Obtener el beta angle del Flyby
%
cc = zeros(1,n_fb);
for j = 1:n_fb
cc(j) = (solution(j).parameter(2));
end

%
% Obtener la velocidad relativa en el Flyby
%
vinf = zeros(1,n_fb+1);
%
for j = 1:n_fb+1
%    
et0_min    = cspice_str2et(setup.Initial_Date);
et0_max    = cspice_str2et(setup.Final_Date);
et0_factor = t0;
et0        = et0_min + (et0_max - et0_min)*et0_factor; 
%
et = et0 + solution(j).time(end)*tc;
%
v = solution(j).state(end,4:6)';    
S1  = cspice_spkezr(fb_b(j), et, 'ECLIPJ2000', 'NONE', 'SUN');
vp = S1(4:6)*1e3/vc;    
v_inf1  = v - vp;
vinf(j) = norm(v_inf1)*vc/1e3;
%
end
%
% Obtain the DV for each phase
%
DV = zeros(1,n_fb+1);
%
for j = 1:n_fb+1
%    
DV(j) = -9.81*3*log(solution(j).state(end,7)/solution(j).state(1,7));
%
end
%
% Get maximum acceleration
%
amax = zeros(1,n_fb+1);
%
for j = 1:n_fb+1
    %
    T = solution(j).control(:,1);
    %
    x = [ solution(j).state(:,1)];
    y = [ solution(j).state(:,2)];
    z = [ solution(j).state(:,3)];
    %
    r = sqrt( x.^2 + y.^2 + z.^2 );
    %
    % Thrust Acceleration
    %
    P0 = 10;
    P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r);
    P = (2.6 *(P>2.6) + P.*(P<=2.6)) ;
    %T = T.*(T>0.5);
    TT = T.*(-1.9137 + 36.242 * P)/300/1000 .* (P>0.649);
    amax(j) = mean((TT));
end
%
%fprintf( fid, '%7.5f ',Time) ;
%
fprintf( fid, '%7.5f ',1-solution(n_fb+1).state(end,7)) ;
%
fprintf( fid, '%7.5f ',t0) ;
%
fprintf( fid, '%7.5f ',t) ;
%
fprintf( fid, '%7.5f ',rfb) ;
%
fprintf( fid, '%7.5f ',vinf) ;
%
fprintf( fid, '%7.5f ',DV) ;
%
fprintf( fid, '%7.5f ',amax) ;
%
fprintf( fid, '%7.5f ',cc*180/pi) ;
%
fprintf( fid, '\n') ;
%
end