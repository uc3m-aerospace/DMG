clear setup limits guess

% Phase 1 Information
iphase = 1;
limits(iphase).nodes = 50;
limits(iphase).time.min = [0 10];
limits(iphase).time.max = [0 10];
limits(iphase).state.min(1,:) = [1 0 0];
limits(iphase).state.max(1,:) = [1 8 8];
limits(iphase).state.min(2,:) = [0.1 0 0];
limits(iphase).state.max(2,:) = [0.1 8 8];
limits(iphase).state.min(3,:) = [40 0 0];
limits(iphase).state.max(3,:) = [40 45 45];
limits(iphase).state.min(4,:) = [0 0 0];
limits(iphase).state.max(4,:) = [0 8 8];
limits(iphase).state.min(5,:) = [0 0 0];
limits(iphase).state.max(5,:) = [0 8 8];
limits(iphase).state.min(6,:) = [1 0 0];
limits(iphase).state.max(6,:) = [1 8 8];
limits(iphase).state.min(7,:) = [0 0 0];
limits(iphase).state.max(7,:) = [0 8 8];
limits(iphase).state.min(8,:) = [0 0 0];
limits(iphase).state.max(8,:) = [1 1 1];
limits(iphase).state.min(9,:) = [0 0 0];
limits(iphase).state.max(9,:) = [1 1 1];
limits(iphase).control.min(1,:) = -10;
limits(iphase).control.max(1,:) = 10;
limits(iphase).control.min(2,:) = -10;
limits(iphase).control.max(2,:) = 10;
limits(iphase).parameter.min = [];
limits(iphase).parameter.max = [];
limits(iphase).path.min = [];
limits(iphase).path.max = [];
limits(iphase).event.min = [];
limits(iphase).event.max = [];
limits(iphase).duration.min = [];
limits(iphase).duration.max = [];
guess(iphase).time = [0; 10];
guess(iphase).state(:,1) = [1; 4];
guess(iphase).state(:,2) = [.1; 7];
guess(iphase).state(:,3) = [40; 40];
guess(iphase).state(:,4) = [0; 1];
guess(iphase).state(:,5) = [0; 1];
guess(iphase).state(:,6) = [1; 1];
guess(iphase).state(:,7) = [0; 0];
guess(iphase).state(:,8) = [0; 1];
guess(iphase).state(:,9) = [0; 1];
guess(iphase).control(:,1) = [0; 0];
guess(iphase).control(:,2) = [0; 0];
guess(iphase).parameter    = [];

linkages = [];
setup.name = 'Lee-Ramirez-Bioreactor-Problem';
setup.funcs.cost = 'leeBioreactorCost';
setup.funcs.dae = 'leeBioreactorDae';
setup.limits = limits;
setup.guess = guess;
setup.linkages = linkages;
setup.derivatives = 'automatic';
setup.autoscale = 'off';
setup.solver ='ipopt';
setup.method ='hermite-simpson';


output = DMG(setup);

solution = output.solution;
