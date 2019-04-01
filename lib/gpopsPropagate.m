function [prop,prop_error] = gpopsPropagate(output,phase,solver,options)
%------------------------------------------------------------------%
%          Propagate Trajectory with Specified Controls            %
%------------------------------------------------------------------%
%--------%
% Inputs:
%--------%
%    output:    GPOPS output structure
%    phase:     phase number(s) in which to perform propagation 
%               (default = all)
%    solver:    string containing name of MATLAB ODE solver 
%               (default = ode45)
%    options:   options from the MATLAB command ODESET
%--------%
% Outputs:
%--------%
%    prop:          Propagated solution from ODE solver
%    prop_error:    Error between GPOPS and propagted trajectory
%--------------------%
% Calling Sequence 1:
%--------------------%
%  [PROP,PROP_ERROR] = gpopsPropagate(OUTPUT)
%      Will propagate entire solution from initial condition of FIRST phase
%      of the optimal control problem using default solver
%--------------------%
% Calling Sequence 2:
%--------------------%
%  [PROP,PROP_ERROR] = gpopsPropagate(OUTPUT,PHASE)
%      Will propagate solution from initial condition in specified phases
%      of the optimal control problem using default solver
% --------------------%
% Calling Sequence 3:
% --------------------%
%  [PROP,PROP_ERROR] = gpopsPropagate(OUTPUT,PHASE,SOLVER,OPTIONS)
%      Will propagate solution from initial condition in specified phases
%      of the optimal control problem using the solver and options given in
%      the third and fourth arguments.
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

% ------------------------------ %
% Check Number of Input Arguments
% ------------------------------ %
if nargin == 1
    phase = [];
    solver = 'ode45';
    options = [];
elseif nargin == 2
    solver = 'ode45';
    options = [];
elseif nargin == 3 || nargin > 4
    error('Invalid number of Input Arguments');
end
% ------------------------------ %
% Check Output structure
% ------------------------------ %
if isfield(output,'solution')
    if iscell(output.solution)
        output = gpopsConvertInput(output);
        output = ConvertSolution(output);
    end
    solution = output.solution;
else
    error('Solution field not found in output structure')
end
if isfield(output,'funcs') && isfield(output.funcs,'dae')
    if isa(output.funcs.dae,'char'),
        dynamics = str2func(output.funcs.dae);
    elseif isa(output.funcs.dae,'function_handle')
        dynamics = output.funcs.dae;
    else
        error('Invalid DAE function in output structure')
    end
else
    error('DAE function not found in output structure')
end
%----------------------%
% Get Number of Phases %
%----------------------%
n_phases = length(solution);

findInitCondFlag = 0;
if isempty(phase)
    phases_to_propagate = 1:n_phases;
    findInitCondFlag = 1;
else
    phases_to_propagate = phase;
    if any(phase > n_phases) || any(phase < 1)
        error('Number of phases to integrate exceeds phases in problem');
    end
end
%----------------------------------%
% Propagate Results for Each Phase %
%----------------------------------%
n_phases = length(phases_to_propagate);
prop  = repmat(struct('phase',[],'time',[],'state',[],'control',[],'parameter',[]),n_phases,1);
prop_error = repmat(struct('phase',[],'time',[],'state',[]),n_phases,1);
for i = phases_to_propagate
    
    itime = solution(i).time;
    if findInitCondFlag && i > 1
        icond = findInitCond(output,i,yout(end,:));
        fprintf('Propagating solution in phase %i using Final conditions of phase %i\n',i,i-1)
    else
        icond = solution(i).state(1,:);
        fprintf('Propagating solution in phase %i using Initial conditions of phase %i\n',i,i)
    end
    icontrols = solution(i).control;
    iparameter = solution(i).parameter;
    nstates   = size(solution(i).state,2);
    switch lower(solver)
     case 'ode45'
      [tout,yout] = ode45(@gpopsDynamics,itime,icond,options,icontrols,iparameter,i,itime,dynamics,nstates);
      
     case 'ode113'
      [tout,yout] = ode113(@gpopsDynamics,itime,icond,options,icontrols,iparameter,i,itime,dynamics,nstates);
     case 'ode15s'
      [tout,yout] = ode15s(@gpopsDynamics,itime,icond,options,icontrols,iparameter,i,itime,dynamics,nstates);
     otherwise
      str = strcat(['ODE solver ',solver,' does not exist or is not supported']);
      error(str);
    end
    prop(i).phase = i;
    prop(i).time = tout;
    prop(i).state = yout;
    prop(i).control = icontrols;
    prop(i).parameter = iparameter;
    prop_error(i).phase = i;
    prop_error(i).time = tout;
    prop_error(i).state = yout - solution(i).state;
end

if isfield(output,'convertOutput')
    prop = ConvertProp(prop);
    prop_error = ConvertProp(prop_error);
end
return


function xdot = gpopsDynamics(t,x,icontrol,iparameter,i,itime,dynamics,nstates)

%------------------------------------------------------------------%
% GPOPSDYNAMICS:  Dummy ODE function for use with GPOPS            %
%------------------------------------------------------------------%

%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, and Christopher Darby.                %
%------------------------------------------------------------------%

sol.time      = t;
sol.state     = x.';
if isempty(icontrol)
    sol.control = [];
else
    sol.control = interp1(itime,icontrol,t,'spline');
end
sol.parameter = iparameter; 
sol.phase     = i;

dae = feval(dynamics,sol);
xdot = dae(1:nstates);
xdot = xdot.';
    
return


%------------------------------------------------------------------%
% FINDINITCOND:  find inital condition of current phase            %
%------------------------------------------------------------------%
function icond = findInitCond(output,iphase,stateL)
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, and Christopher Darby.                %
%------------------------------------------------------------------%

% check for linkage function
if isfield(output,'funcs') && isfield(output.funcs,'link')
    if isa(output.funcs.link,'char'),
        connection = str2func(output.funcs.link);
    elseif isa(output.funcs.link,'function_handle')
        connection = output.funcs.link;
    else
        error('Invalid Linkage function in output structure')
    end
else
    error('Linkage function not found in output structure')
end

%check for linkage bounds
if isfield(output,'linkages') && isfield(output.linkages,'min') ...
        && isfield(output.linkages,'max')
    % find bounds
    bounds = [];
    for i = 1:length(output.linkages)
        if output.linkages(i).right.phase == iphase ...
            && isequal(output.linkages(i).left.phase, output.linkages(i).right.phase-1)
            %check bounds
            if isequal(output.linkages(i).min,output.linkages(i).max)
                bounds = output.linkages(i).min;
            else
                error(sprintf(['Linkage upper and lower bounds not equal\n',...
                    'Note: problem must have equality phase linkages']))
            end
        end
    end
    if isempty(bounds)
        error(sprintf(['Linkage condition not found in output structure\n',...
            'Note: problem must have sequential phases']))
    end
else
    error('Linkage bounds not found in output structure')
end

% get data from structure
pL = output.solution(iphase-1).parameter;       %left phase parameters
stateR = output.solution(iphase).state(1,:);%right phase inital state (guess)
pR = output.solution(iphase).parameter;         %right phase parameters
if length(bounds) ~= length(stateR)
    error('Number of linkages must be equal to number of states in right phase')
end

% find stateR using Newton-Raphson iteration
tol = 2E-6;
epsilon = 1E-6;
for ii = 1:100
    sol.left.state = stateL;
    sol.left.parameter = pL;
    sol.left.phase = iphase-1;
    sol.right.state = stateR;
    sol.right.parameter = pR;
    sol.right.phase = iphase;
    link_result = feval(connection,sol);
    defect = link_result(:) - bounds;
    if norm(defect,'inf') < tol
        icond = stateR;
        return
    end
    % find Jacobian of defect
    dL_dX = zeros(length(defect),length(stateR));
    for jj = 1:length(stateR)
        DstateR = stateR;
        DstateR(jj) = DstateR(jj) + epsilon;
        Dsol.left.state = stateL;
        Dsol.left.parameter = pL;
        Dsol.left.phase = iphase-1;
        Dsol.right.state = DstateR;
        Dsol.right.parameter = pR;
        Dsol.right.phase = iphase;
        link_result = feval(connection,Dsol);
        Ddefect = link_result(:) - bounds;
        dL_dX(:,jj) = (Ddefect - defect)/epsilon;
    end
    % update stateR
    stateRDelta = -inv(dL_dX)*defect;
    stateR = stateR + stateRDelta.';
end
error('unable to link phases %i and %i',iphase-1,iphase)

return
   
%------------------------------------------------------------------%
% ConvertSolution:  convert solution to structure format           %
%------------------------------------------------------------------%
function output = ConvertSolution(output);

solTemp = output.solution;
output = rmfield(output,'solution');
for iphase = 1:(size(solTemp,1)-1)
    output.solution(iphase).time = solTemp{iphase,1};
    output.solution(iphase).state = solTemp{iphase,2};
    output.solution(iphase).control = solTemp{iphase,3};
    output.solution(iphase).parameter = solTemp{iphase,4};
    output.solution(iphase).costate = solTemp{iphase,5};
    output.solution(iphase).Hamiltonian = solTemp{iphase,6};
    output.solution(iphase).pathmult = solTemp{iphase,7};
    output.solution(iphase).Mayer_cost = solTemp{iphase,8};
    output.solution(iphase).Lagrange_cost = solTemp{iphase,9};
end

return

%------------------------------------------------------------------%
% ConvertProp:  convert propigated solution to cell format         %
%------------------------------------------------------------------%
function propOut = ConvertProp(prop);

propOut = cell(length(prop),2);
for i = 1:length(prop)
    propOut{i,1} = prop(i).time;
    propOut{i,2} = prop(i).state;
end

return
