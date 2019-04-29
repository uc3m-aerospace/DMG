clc
close all
mission = load('mission_performance.txt');

mu_jupiter = 1.26686534e17;                
r_callisto = 1.8827e9;        
r_ganymede = 1.0704e9;                           
time = 2.0044800e7;           
lc = r_ganymede;
tc = sqrt(lc^3/mu_jupiter);
vc = lc/tc;
ac = vc/tc;
r(1,:) = mission(:,2)*1000/lc;
r(2,:) = mission(:,3)*1000/lc;
r(3,:) = 0;
v(1,:) = mission(:,8)*1000/vc;
v(2,:) = mission(:,9)*1000/vc;
v(3,:) = 0;
t(1,:) = mission(:,1)*24*3600/tc;
w = sqrt(mu_jupiter/r_ganymede^3)*tc;%1;%sqrt(lc^3/r_callisto^3); Module angular velocity
omega = [0; 0; w]; 
r_prime = zeros(3,length(mission(:,1)));
v_prime = zeros(3,length(mission(:,1)));
u = zeros(3,length(mission(:,1)));
rot_matrix = zeros(3,3,length(mission(:,1)));
psi = atan2(r(2,1),r(1,1));
for i = 1:numel(r(1,:))
    rot_matrix(:,:,i) = [cos(w*t(i)+psi) sin(w*t(i)+psi) 0; -sin(w*t(i)+psi) cos(w*t(i)+psi) 0; 0 0 1];
    r_prime(:,i) = rot_matrix(:,:,i)*r(:,i);
    v_prime(:,i) = v(:,i) - cross(omega,r(:,i));
    u(:,i) = cross(omega,r_prime(:,i));
%     plot(t(:),u(1,:),'r',t(:),u(2,:),'k')
% %     axis([-2 2 -2 2])
%     pause(.0005)
end

angle_prime = zeros(length(mission(:,1)),1);
T = zeros(2,length(mission(:,1)));
T_prime = zeros(2,length(mission(:,1)));
thrust_angle = zeros(length(mission(:,1)),1);
for i = 1:length(mission(:,1))
    angle_prime(i,1) = atan2(mission(i,3),mission(i,2));
    T(1,i) = -cos(mission(i,6)*pi/180)*sin(angle_prime(i,1)) + sin(mission(i,6)*pi/180)*cos(angle_prime(i,1));
    T(2,i) = cos(mission(i,6)*pi/180)*cos(angle_prime(i,1)) + sin(mission(i,6)*pi/180)*sin(angle_prime(i,1));
    rot_matrix3 = [cos(w*t(i)) sin(w*t(i)); -sin(w*t(i)) cos(w*t(i))];
    T_prime(:,i) = rot_matrix3*T(:,i);
    thrust_angle(i,1) = atan2(T_prime(2,i),T_prime(1,i));
end

% figure
% plot(linspace(0,numel(mission(:,1)),numel(mission(:,1))),t,'r')
% figure
% plot(t,atan2(T(2,:),T(1,:)),'g')

%% Analizando a callisto respecto ganymede
    w2 = sqrt(mu_jupiter/r_callisto^3)*tc;
    rca(1,:) = cos(w2*t)*r_callisto/lc;
    rca(2,:) = sin(w2*t)*r_callisto/lc;
    rca(3,:) = 0;
    
    rga(1,:) = cos(w*t);
    rga(2,:) = sin(w*t);
    rga(3,:) = 0;

    
    rca_prime = zeros(3,length(mission(:,1)));
    rot_matrix2 = zeros(3,3,numel(rca(1,:)));
    
    for i = 1:numel(rca(1,:))
        rot_matrix2(:,:,i) = [cos(w*t(i)+psi) sin(w*t(i)+psi) 0; -sin(w*t(i)+psi) cos(w*t(i)+psi) 0; 0 0 1];
        rca_prime(:,i) = rot_matrix2(:,:,i)*rca(:,i);
        %Movimiento de callisto respecto al sistema no inercial
%         plot(rca_prime(1,:),rca_prime(2,:))
%         axis([-2 2 -2 2])
%         pause(.0005)
    end

%% Plot del movimiento de ambas lunas respecto el sistema inercial
%     rca = zeros(3,length(mission(:,1)));
%     rga = zeros(3,length(mission(:,1)));
% for i=1:length(mission(:,1))
%     rca(1,i) = cos(w2*t(i))*r_callisto/lc;
%     rca(2,i) = sin(w2*t(i))*r_callisto/lc;
%     
%     rga(1,i) = cos(w*t(i));
%     rga(2,i) = sin(w*t(i));
%     plot(rga(1,:),rga(2,:),'b',rca(1,:),rca(2,:),'k')
%     axis([-2 2 -2 2])
%     pause(.0005)
% end
