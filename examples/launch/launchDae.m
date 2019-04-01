function [dae Ddae] = launchDae(sol,setup);

CONSTANTS = setup.CONSTANTS;
t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;
iphase = sol.phase;
r = x(:,1:3);
v = x(:,4:6);
m = x(:,7);

rad = sqrt(sum(r.*r,2));
omega_matrix = CONSTANTS.omega_matrix;
vrel = v-r*omega_matrix.';
speedrel = sqrt(sum(vrel.*vrel,2));

altitude = rad-CONSTANTS.Re;
rho = CONSTANTS.rho0*exp(-altitude/CONSTANTS.H);
bc  = (rho./(2*m)).*CONSTANTS.sa*CONSTANTS.cd;
bcspeed = bc.*speedrel;
% bcspeedmat = [bcspeed bcspeed bcspeed];
bcspeedmat = repmat(bcspeed,1,3);

Drag = -bcspeedmat.*vrel;
muoverradcubed = CONSTANTS.mu./rad.^3;
muoverradcubedmat = [muoverradcubed muoverradcubed muoverradcubed];
grav = -muoverradcubedmat.*r;

if iphase==1,
    T_srb = 6*CONSTANTS.thrust_srb*ones(size(t));
    T_first = CONSTANTS.thrust_first*ones(size(t));
    T_tot = T_srb+T_first;
    m1dot = -T_srb./(CONSTANTS.g0*CONSTANTS.ISP_srb);
    m2dot = -T_first./(CONSTANTS.g0*CONSTANTS.ISP_first);
    mdot = m1dot+m2dot;
elseif iphase==2,
    T_srb = 3*CONSTANTS.thrust_srb*ones(size(t));
    T_first = CONSTANTS.thrust_first*ones(size(t));
    T_tot = T_srb+T_first;
    m1dot = -T_srb./(CONSTANTS.g0*CONSTANTS.ISP_srb);
    m2dot = -T_first./(CONSTANTS.g0*CONSTANTS.ISP_first);
    mdot = m1dot+m2dot;    
elseif iphase==3
    T_first = CONSTANTS.thrust_first*ones(size(t));
    T_tot = T_first;
    mdot = -T_first./(CONSTANTS.g0*CONSTANTS.ISP_first);
elseif iphase==4,
    T_second = CONSTANTS.thrust_second*ones(size(t));
    T_tot = T_second;
    mdot = -T_second./(CONSTANTS.g0*CONSTANTS.ISP_second);
end;

path = sum(u.*u,2);
Toverm = T_tot./m;
Tovermmat = [Toverm Toverm Toverm];
thrust = Tovermmat.*u;

rdot = v;
vdot = thrust+Drag+grav;

dae = [rdot vdot mdot path];

% avoid calc of derivs in not necessary
if nargout == 2 

    % to eliminate divide by zero in analytic deriv calc
    speedrel(logical(speedrel == 0)) = 1;    

    Ddae = zeros(8*length(t),11);
    N = length(t);  %number of nodes

    % drdot/dx
    Ddae(1:N,4) = 1;        % drdot1/dv1
    Ddae(N+1:2*N,5) = 1;    % drdot2/dv2
    Ddae(2*N+1:3*N,6) = 1;  % drdot3/dv3

    % dvdot/dx
    dDrag1_dr1 = bc.*vrel(:,2).*vrel(:,1)./speedrel*CONSTANTS.omega_matrix(2,1) ...
        + bc.*speedrel.*vrel(:,1).*r(:,1)./rad./CONSTANTS.H;
    dDrag1_dr2 = -bc.*vrel(:,1).*vrel(:,1)./speedrel*CONSTANTS.omega_matrix(2,1) ...
        + bc.*speedrel.*vrel(:,1)./CONSTANTS.H.*r(:,2)./rad ...
        - bc.*speedrel*CONSTANTS.omega_matrix(2,1);
    dDrag1_dr3 = bc.*speedrel.*vrel(:,1)./CONSTANTS.H.*r(:,3)./rad;
    dDrag2_dr1 = bc.*vrel(:,2).*vrel(:,2)./speedrel*CONSTANTS.omega_matrix(2,1) ...
        + bc.*speedrel.*vrel(:,2)./CONSTANTS.H.*r(:,1)./rad ...
        + bc.*speedrel*CONSTANTS.omega_matrix(2,1);
    dDrag2_dr2 = -bc.*vrel(:,1).*vrel(:,2)./speedrel*CONSTANTS.omega_matrix(2,1) ...
        + bc.*speedrel.*vrel(:,2)./CONSTANTS.H.*r(:,2)./rad;
    dDrag2_dr3 = bc.*speedrel.*vrel(:,2)./CONSTANTS.H.*r(:,3)./rad;
    dDrag3_dr1 = bc.*vrel(:,2).*vrel(:,3)./speedrel*CONSTANTS.omega_matrix(2,1) ...
        + bc.*speedrel.*vrel(:,3)./CONSTANTS.H.*r(:,1)./rad;
    dDrag3_dr2 = -bc.*vrel(:,1).*vrel(:,3)./speedrel*CONSTANTS.omega_matrix(2,1) ...
        + bc.*speedrel.*vrel(:,3)./CONSTANTS.H.*r(:,2)./rad;
    dDrag3_dr3 = bc.*speedrel.*vrel(:,3)./CONSTANTS.H.*r(:,3)./rad;

    dgrav1_dr1 = -muoverradcubed + 3*CONSTANTS.mu.*r(:,1).^2./rad.^5;
    dgrav1_dr2 = 3*CONSTANTS.mu.*r(:,1).*r(:,2)./rad.^5;
    dgrav1_dr3 = 3*CONSTANTS.mu.*r(:,1).*r(:,3)./rad.^5;
    dgrav2_dr1 = 3*CONSTANTS.mu.*r(:,2).*r(:,1)./rad.^5;
    dgrav2_dr2 = -muoverradcubed + 3*CONSTANTS.mu.*r(:,2).^2./rad.^5;
    dgrav2_dr3 = 3*CONSTANTS.mu.*r(:,2).*r(:,3)./rad.^5;
    dgrav3_dr1 = 3*CONSTANTS.mu.*r(:,3).*r(:,1)./rad.^5;
    dgrav3_dr2 = 3*CONSTANTS.mu.*r(:,3).*r(:,2)./rad.^5;
    dgrav3_dr3 = -muoverradcubed + 3*CONSTANTS.mu.*r(:,3).^2./rad.^5;

    Ddae(3*N+1:4*N,1) = dDrag1_dr1 + dgrav1_dr1;    % dvdot1/dr1
    Ddae(3*N+1:4*N,2) = dDrag1_dr2 + dgrav1_dr2;    % dvdot1/dr2
    Ddae(3*N+1:4*N,3) = dDrag1_dr3 + dgrav1_dr3;    % dvdot1/dr3
    Ddae(4*N+1:5*N,1) = dDrag2_dr1 + dgrav2_dr1;    % dvdot2/dr1
    Ddae(4*N+1:5*N,2) = dDrag2_dr2 + dgrav2_dr2;    % dvdot2/dr2
    Ddae(4*N+1:5*N,3) = dDrag2_dr3 + dgrav2_dr3;    % dvdot2/dr3
    Ddae(5*N+1:6*N,1) = dDrag3_dr1 + dgrav3_dr1;    % dvdot3/dr1
    Ddae(5*N+1:6*N,2) = dDrag3_dr2 + dgrav3_dr2;    % dvdot3/dr2
    Ddae(5*N+1:6*N,3) = dDrag3_dr3 + dgrav3_dr3;    % dvdot3/dr3

    dDrag1_dv1 = -bc.*speedrel - bc.*vrel(:,1).^2./speedrel;
    dDrag1_dv2 = -bc.*vrel(:,1).*vrel(:,2)./speedrel;
    dDrag1_dv3 = -bc.*vrel(:,1).*vrel(:,3)./speedrel;
    dDrag2_dv1 = -bc.*vrel(:,2).*vrel(:,1)./speedrel;
    dDrag2_dv2 = -bc.*speedrel - bc.*vrel(:,2).^2./speedrel;
    dDrag2_dv3 = -bc.*vrel(:,2).*vrel(:,3)./speedrel;
    dDrag3_dv1 = -bc.*vrel(:,3).*vrel(:,1)./speedrel;
    dDrag3_dv2 = -bc.*vrel(:,3).*vrel(:,2)./speedrel;
    dDrag3_dv3 = -bc.*speedrel - bc.*vrel(:,3).^2./speedrel;

    Ddae(3*N+1:4*N,4) = dDrag1_dv1;    % dvdot1/dv1
    Ddae(3*N+1:4*N,5) = dDrag1_dv2;    % dvdot1/dv2
    Ddae(3*N+1:4*N,6) = dDrag1_dv3;    % dvdot1/dv3
    Ddae(4*N+1:5*N,4) = dDrag2_dv1;    % dvdot2/dv1
    Ddae(4*N+1:5*N,5) = dDrag2_dv2;    % dvdot2/dv2
    Ddae(4*N+1:5*N,6) = dDrag2_dv3;    % dvdot2/dv3
    Ddae(5*N+1:6*N,4) = dDrag3_dv1;    % dvdot3/dv1
    Ddae(5*N+1:6*N,5) = dDrag3_dv2;    % dvdot3/dv2
    Ddae(5*N+1:6*N,6) = dDrag3_dv3;    % dvdot3/dv3

    dDrag1_dm = -Drag(:,1)./m;
    dDrag2_dm = -Drag(:,2)./m;
    dDrag3_dm = -Drag(:,3)./m;

    Ddae(3*N+1:4*N,7) = dDrag1_dm - thrust(:,1)./m; % dvdot1/dm
    Ddae(4*N+1:5*N,7) = dDrag2_dm - thrust(:,2)./m; % dvdot2/dm
    Ddae(5*N+1:6*N,7) = dDrag3_dm - thrust(:,3)./m; % dvdot3/dm

    %dvdot/du
    Ddae(3*N+1:4*N,8) = Toverm;  % dvdot1/du1
    Ddae(4*N+1:5*N,9) = Toverm;  % dvdot2/du2
    Ddae(5*N+1:6*N,10) = Toverm; % dvdot3/du3

    % mass dynamics independant of State
    % Ddae(6*N+1:7*N,:) = 0

    %dpath/du
    Ddae(7*N+1:8*N,8) = 2*u(:,1);  % dp/du1
    Ddae(7*N+1:8*N,9) = 2*u(:,2);  % dp/du2
    Ddae(7*N+1:8*N,10) = 2*u(:,3); % dp/du3
end

