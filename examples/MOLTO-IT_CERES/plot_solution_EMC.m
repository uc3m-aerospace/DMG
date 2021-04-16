%
lc  = 149597870.700e03;
mu  = 132712440018e09;
tc  = sqrt(lc^3/mu);
vc  = lc/tc;
ac  = vc/tc;
%
t_end = 0;
figure
hold on
%
t0 = '2003-06-03';
%
time0 = 0;
for jj = 1:2
    
    initial_guess = load(['Trajectory_Leg',num2str(jj),'.output']);
    t_guess     = initial_guess(:,1)+ time0;
    r_guess     = initial_guess(:,2);
    v_guess     = initial_guess(:,3);
    psi_guess   = initial_guess(:,4);
    T_guess     = initial_guess(:,6);
    alpha_guess = initial_guess(:,5);
    theta_guess = initial_guess(:,7);
    
    dtime =  datetime(t0) + seconds(t_guess*tc);
    plot(dtime,T_guess*ac)
    time0 = t_guess(end);
end

t0 = '2003-05-13';

for jj = 1:2
    time    = [solution(jj).time];
    T       = [solution(jj).control(:,1)];
    x       = [solution(jj).state(:,1)];
    y       = [solution(jj).state(:,2)];
    z       = [solution(jj).state(:,3)];
    %
    %
    m       = [ solution(jj).state(:,7)];
    %
    r = sqrt( x.^2 + y.^2 + z.^2 );
    %
    % Thrust Acceleration
    %
    P0 = setup.P0;
    P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r);
    P = 2.6 *(P>2.6) + P.*(P<=2.6) ;
    TT = T.*(-1.9137 + 36.242 * P)/(setup.m0*ac)/1000.* (P>0.649)./m;
    
    dtime =  datetime(t0) + seconds(time*tc);
    
    plot(dtime,TT*ac)
end
%
figure
hold on
t0 = '2003-06-03';
%
time0 = 0;
for jj = 1:2
    
    initial_guess = load(['Trajectory_Leg',num2str(jj),'.output']);
    t_guess     = initial_guess(:,1)+time0;
    r_guess     = initial_guess(:,2);
    v_guess     = initial_guess(:,3);
    psi_guess   = initial_guess(:,4);
    T_guess     = initial_guess(:,6);
    alpha_guess = initial_guess(:,5);
    if jj == 2
    alpha_guess = initial_guess(:,5)+pi;
    end
    theta_guess = initial_guess(:,7);
    T_guess      = T_guess>0;
    %
    swicth_point = find(not(T_guess(1:end-1) == T_guess(2:end)));
    %
    swicth_point_ext = [1;swicth_point;numel(T_guess) ];
    %
    dtime =  datetime(t0) + seconds(t_guess*tc);
    %
    for kk = 1:numel(swicth_point)+1
        ind1 = swicth_point_ext(kk);
        ind2 = swicth_point_ext(kk+1);
        if T_guess(ind1+1)== 1
            plot(dtime(ind1+1:ind2),alpha_guess(ind1+1:ind2)*180/pi,'- black')
            
        end
    end
    time0 = t_guess(end);
    
end

t0 = '2003-05-13';

for jj = 1:2
    time    = [solution(jj).time];
    T       = [solution(jj).control(:,1)];
    T       = T>0.2;
    alpha   = [solution(jj).control(:,2)];
    x       = [solution(jj).state(:,1)];
    y       = [solution(jj).state(:,2)];
    z       = [solution(jj).state(:,3)];
    %
    %
    m       = [ solution(jj).state(:,7)];
    %
    r = sqrt( x.^2 + y.^2 + z.^2 );
    %
    % Thrust Acceleration
    %
    P0 = setup.P0;
    P  = P0 ./ r.^2 .* (1.1063 + 0.1495./r -0.299./r.^2 ) ./ (1-0.0432*r);
    P = 2.6 *(P>2.6) + P.*(P<=2.6) ;
    TT = T.*(-1.9137 + 36.242 * P)/(setup.m0*ac)/1000.* (P>0.649)./m;
    
    swicth_point = find(not(T(1:end-1) == T(2:end)));
    %
    swicth_point_ext = [1;swicth_point;numel(T) ];
    %
    dtime =  datetime(t0) + seconds(time*tc);
    %
    for kk = 1:numel(swicth_point)+1
        ind1 = swicth_point_ext(kk);
        ind2 = swicth_point_ext(kk+1);
        if T(ind1+1)== 1
            plot(dtime(ind1:ind2),alpha(ind1:ind2)*180/pi,'- blue')
            
        end
    end
    
end





