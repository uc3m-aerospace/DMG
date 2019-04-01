% Minimum Time-to-Climb Problem 
% The vehicle model for this problem is taken 
% from the following two references:
% Seywald, H., Clifs, E. M., and Well, K. H., 
% "Range Optimal Trajectories for an Aircraft Flying in 
%  the Vertical Plane," Journal of Guidance, Control, and Dynamics,
%  Vol. 17, No. 2, March-April, 1994.
%
%  Rao, A. V., Extension of a Computational Singular Perturbation
%  Methodology to Optimal Control Problems, Ph.D. Thesis, Dept. of
%  Mechanical and Aerospace Engineering, Princeton University,
%  June 1996.

clear setup limits guess CONSTANTS
global CONSTANTS
CoF(1,:) = [2.61059846050e-2;
            -8.57043966269e-2;
            1.07863115049e-1;
            -6.44772018636e-2;
            1.64933626507e-2;
            0];
CoF(2,:) = [1.37368651246e0;
            -4.57116286752e0;
            5.72789877344e0;
            -3.25219000620e0;
            7.29821847445e-1;
            0];
CoF(3,:) = [1.23001735612e0;
            -2.97244144190e0;
            2.78009092756e0;
            -1.16227834301e0;
            1.81868987624e-1;
            0];
CoF(4,:) = [1.42392902737e1;
            -3.24759126471e1;
            2.96838643792e1;
            -1.33316812491e1;
            2.87165882405e0;
            -2.27239723756e-1];
CoF(5,:) = [0.11969995703e6;
            -0.14644656421e5;
            -0.45534597613e3;
            0.49544694509e3;
            -0.46253181596e2;
            0.12000480258e1];
CoF(6,:) = [-0.35217318620e6;
            0.51808811078e5;
            0.23143969006e4;
            -0.22482310455e4;
            0.20894683419e3;
            -0.53807416658e1];
CoF(7,:) = [0.60452159152e6;
            -0.95597112936e5;
            -0.38860323817e4;
            0.39771922607e4;
            -0.36835984294e3;
            0.94529288471e1];
CoF(8,:) = [-0.43042985701e6;
            0.83271826575e5;
            0.12357128390e4;
            -0.30734191752e4;
            0.29388870979e3;
            -0.76204728620e1];
CoF(9,:) = [0.13656937908e6;
            -0.32867923740e5;
            0.55572727442e3;
            0.10635494768e4;
            -0.10784916936e3;
            0.28552696781e1];
CoF(10,:) = [-0.16647992124e5;
            0.49102536402e4;
            -0.23591380327e3;
            -0.13626703723e3;
            0.14880019422e2;
            -0.40379767869e0];
CoFZ = [-3.48643241e-2;
        3.50991865e-3;
        -8.33000535e-5;
        1.15219733e-6];

CONSTANTS.CoF = CoF;
CONSTANTS.CoFZ = CoFZ;

g = 9.80665;
m = 37000/2.2;
feettometer = .3048;

h0 = 0*feettometer;
hf = 65600*feettometer;
v0 = 424.26*feettometer;
vf = 968.148*feettometer;
e0 = (v0^2/(2*g)+h0);
ef = (vf^2/(2*g)+hf);
fpa0 = 0;
fpaf = 0;

hmin =  0*feettometer;
hmax =  69000*feettometer;
vmin =  1*feettometer;
vmax =  2000*feettometer;
emin = (vmin^2/(2*g)+hmin);
emax = (vmax^2/(2*g)+hmax);
fpamin = -40/180*pi;
fpamax = -fpamin;
umin = -10;
umax =  10;
t0min = 0;
t0max = 0;
tfmin = 100;
tfmax = 350;

% Phase 1 Information
iphase = 1;
limits(iphase).nodes = 50;
limits(iphase).time.min = [t0min tfmin];
limits(iphase).time.max = [t0max tfmax];
limits(iphase).state.min(1,:) = [h0 hmin hf];
limits(iphase).state.max(1,:) = [h0 hmax hf];
limits(iphase).state.min(2,:) = [e0 emin ef];
limits(iphase).state.max(2,:) = [e0 emax ef];
limits(iphase).state.min(3,:) = [fpa0 fpamin fpaf];
limits(iphase).state.max(3,:) = [fpa0 fpamax fpaf];
limits(iphase).control.min = umin;
limits(iphase).control.max = umax;
limits(iphase).parameter.min = [];
limits(iphase).parameter.max = [];
limits(iphase).path.min = [];
limits(iphase).path.max = [];
limits(iphase).event.min = [];
limits(iphase).event.max = [];
limits(iphase).duration.min = [];
limits(iphase).duration.max = [];
guess(iphase).time = [t0min; tfmax];
guess(iphase).state(:,1) = [hf/2; hf/2];
guess(iphase).state(:,2) = [ef/2; ef/2];
guess(iphase).state(:,3) = [fpamax/2; fpamax/2];
guess(iphase).control = [0; 0];
guess(iphase).parameter = []; % No parameters in Phase 1

setup.name  = 'Minimum-Time-to-Climb-Problem';
setup.funcs.cost = 'minimumClimbCost';
setup.funcs.dae = 'minimumClimbDae';
setup.funcs.link = '';
setup.limits = limits;
setup.guess = guess;

setup.autoscale ='on';
setup.derivatives ='automatic';
setup.solver ='snopt';
setup.parallel = 'no';
setup.method ='pseudospectral';
setup.CONSTANTS = CONSTANTS;

output = DMG(setup);

solution = output.solution;

