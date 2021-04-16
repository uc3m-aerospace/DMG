 %
% Plot Trajectory
%
n_fb = numel(setup.flyby_planet);
%
figure(1)
hold on
grid on
axis equal
%
t0 = solution(1).parameter(end);
%
et0_min    = cspice_str2et(setup.Initial_Date);
et0_max    = cspice_str2et(setup.Final_Date);
et0_factor = t0;
et0        = et0_min + (et0_max - et0_min)*et0_factor;
%
n_fb = 3;
%
kk = 1;
%
for i = 1:n_fb + 1
    %
    
    x     = solution(i).state(:,1);
    y     = solution(i).state(:,2);
    z     = solution(i).state(:,3);
    time  = solution(i).time*setup.tc;
    mass  = solution(i).state(:,7);
    %
    plot3(x(1),y(1),z(1),'o')
    plot3(x(end),y(end),z(end),'o')
    %
    T     = solution(i).control(:,1);
    T     = T>0.4;
    %
    swicth_point = find(not(T(1:end-1) == T(2:end)));
    %
    swicth_point_ext = [1;swicth_point;numel(T) ];
    %
    for jj = 1:numel(swicth_point)+1
        ind1 = swicth_point_ext(jj);
        ind2 = swicth_point_ext(jj+1);
        plot3(x(ind1),y(ind1),z(ind1),'o')
        plot3(x(ind2),y(ind2),z(ind2),'o')
        switch T(ind1+1)
            
            case 0
                dtime =  datetime('2000-01-01 12:00:00') +seconds(et0 + time(ind1:ind2));
                datetime(dtime)
                xx = x(ind1:ind2);
                yy = y(ind1:ind2);
                zz = z(ind1:ind2);
                plot3(xx,yy,zz,'. black')
                MJD   = juliandate(dtime)
                
                A = [MJD,xx*setup.lc/1000,yy*setup.lc/1000,zz*setup.lc/1000];
                
                dlmwrite(['EMCcoast',num2str(kk),'.txt'],A, 'precision', 15, 'delimiter','\t')
         
                
            case 1
                
                plot3(x(ind1:ind2),y(ind1:ind2),z(ind1:ind2),'- black','LineWidth',1.5)
                                dtime =  datetime('2000-01-01 12:00:00') +seconds(et0 + time(ind1:ind2));
                datetime(dtime)
                xx = x(ind1:ind2);
                yy = y(ind1:ind2);
                zz = z(ind1:ind2);
                MJD   = juliandate(dtime);
                
                A = [MJD,xx*setup.lc/1000,yy*setup.lc/1000,zz*setup.lc/1000];
                
                dlmwrite(['EMCthrust',num2str(kk),'.txt'],A, 'precision', 15, 'delimiter','\t')
                
        end
        kk = kk+1;
    end
    
    time       = et0 + time;
    
    dtime =  datetime('2000-01-01 00:00:00') + seconds(time);
    datetime(dtime);
    time1 = datetime(dtime(1))
    time2 = datetime(dtime(end))
    m1    = m0*mass(1)
    m2    = m0*mass(end)
    
end