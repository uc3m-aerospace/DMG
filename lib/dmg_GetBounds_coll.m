function setup = dmg_GetBounds_coll(setup)
%------------------------------------------------------------------%
% Get all bounds in a multiple-phase optimal control problem       %
%------------------------------------------------------------------%
%
%------------------------------------------------------------------%

limits = setup.limits;
linkages = setup.linkages;
sizes = setup.sizes;
numphases = setup.numphases;
nodes = horzcat(limits(:).nodes);
variable_offset = 0;
constraint_offset = 0;
nlplimits = cell(numphases,4);
variables = zeros(1,numphases);
constraints = zeros(1,numphases);
variable_indices = cell(1,numphases); 
constraint_indices = cell(1,numphases);
indices = repmat(struct('state',[],'control',[],'time',[],'parameter',[]),numphases,1);

for iphase=1:numphases
    % ---------------------------------------------------------
    % Get the lower and upper limits on the initial and terminal
    % time in the current phase
    % ---------------------------------------------------------
    tMin = limits(iphase).time.min;
    tMax = limits(iphase).time.max;
    t0_min = tMin(1);
    t0_max = tMax(1);
    tf_min = tMin(2);
    tf_max = tMax(2);
    if size(tMin(:),1) ~= 2 || size(tMax(:),1) ~= 2
        error('Time Bound Vectors must have length 2, in phase: %i',iphase)
    end
    if any(isnan(tMin)) || any(isnan(tMax))
        error('Time bounds NaN in phase: %i',iphase)
    end
    % -------------------------------------------------------------------
    % Get the lower and upper limits on the states in the current phase
    % -------------------------------------------------------------------
    state_matrix_min = zeros(nodes(iphase),sizes(iphase,1));
    state_matrix_max = zeros(nodes(iphase),sizes(iphase,1));
    if ~isequal(sizes(iphase,1),0)
        state0Min = limits(iphase).state.min(:,1);
        stateMin = limits(iphase).state.min(:,2);
        statefMin = limits(iphase).state.min(:,3);
        state0Max = limits(iphase).state.max(:,1);
        stateMax = limits(iphase).state.max(:,2);
        statefMax = limits(iphase).state.max(:,3);
        state_matrix_min(1,:) = state0Min;
        state_matrix_min(2:nodes(iphase)-1,:) = repmat(stateMin,1,nodes(iphase)-2).';
        state_matrix_min(nodes(iphase),:) = statefMin;
        state_matrix_max(1,:) = state0Max;
        state_matrix_max(2:nodes(iphase)-1,:) = repmat(stateMax,1,nodes(iphase)-2).';
        state_matrix_max(nodes(iphase),:) = statefMax;
        % -------------------------------------------------------------------
        % Check the lower and upper limits on the states in the current phase
        % ------------------------------------------------------------------- 
        if (any(state0Max - stateMin < 0) || any(statefMax - stateMin < 0) || any(stateMax - state0Min < 0) || any(stateMax - statefMin < 0) || any(stateMax(:,:) - stateMin(:,:) < 0))
            error('State bounds inconsistant (i.e. max < min) in phase: %i',iphase)
        end
        if any(any(isnan(stateMin))) || any(any(isnan(stateMax)))
            error('State bounds NaN in phase: %i',iphase)
        end
    else
        state_matrix_min = [];
        state_matrix_max = [];
    end
    % -------------------------------------------------------------------
    % Get the lower and upper limits on the controls in the current phase
    % -------------------------------------------------------------------
    if ~isequal(sizes(iphase,2),0)
        controlMin = limits(iphase).control.min;
        controlMax = limits(iphase).control.max;
        control_matrix_min = repmat(controlMin,1,nodes(iphase)).';
        control_matrix_max = repmat(controlMax,1,nodes(iphase)).';
        % -------------------------------------------------------------------
        % Check the lower and upper limits on the controls in the current phase
        % ------------------------------------------------------------------- 
        if any(controlMax(:,:) - controlMin(:,:) < 0)
            error('Control bounds inconsistent (i.e. max < min) in phase: %i',iphase)
        end
        if any(isnan(controlMin)) || any(isnan(controlMax))
            error('Control bounds NaN in phase: %i',iphase)
        end
    else
        control_matrix_min = [];
        control_matrix_max = [];
    end
    
    % ----------------------------------------------------------------------------
    % Get the lower and upper limits on the static parameters in the current phase
    % ----------------------------------------------------------------------------
    if ~isequal(sizes(iphase,3),0)
        parameter_min = limits(iphase).parameter.min;
        parameter_max = limits(iphase).parameter.max;
        % -------------------------------------------------------------------
        % Check the lower and upper limits on the static parameters in the current phase
        % ------------------------------------------------------------------- 
        if any(parameter_max(:,:) - parameter_min(:,:) < 0)
            error('Parameter bounds inconsistant (i.e. max < min) in phase: %i',iphase)
        end
        if any(isnan(parameter_min)) || any(isnan(parameter_max))
            error('Parameter bounds NaN in phase: %i',iphase)
        end
    else
        parameter_min = [];
        parameter_max = [];
    end
    % ---------------------------------------------------------------------------
    % Get the lower and upper limits on the path constraints in the current phase
    % ---------------------------------------------------------------------------
    if ~isequal(sizes(iphase,4),0)
        pathMin = limits(iphase).path.min;
        pathMax = limits(iphase).path.max;
        path_matrix_min = repmat(pathMin,1,nodes(iphase)).';
        path_matrix_max = repmat(pathMax,1,nodes(iphase)).';
        % -------------------------------------------------------------------
        % Check the lower and upper limits on the path constraints in the current phase
        % ------------------------------------------------------------------- 
        if any(pathMax(:,:) - pathMin(:,:) < 0)
            error('Path constraint bounds inconsistant (i.e. max < min) in phase: %i',iphase)
        end
        if any(isnan(pathMin)) || any(isnan(pathMax))
            error('Path bounds NaN in phase: %i',iphase)
        end
    else
        path_matrix_min = [];
        path_matrix_max = [];
    end
    % ----------------------------------------------------------------------------
    % Get the lower and upper limits on the event constraints in the current phase
    % ----------------------------------------------------------------------------
    if ~isequal(sizes(iphase,5),0)
        event_vector_min = limits(iphase).event.min;
        event_vector_max = limits(iphase).event.max;
        % -------------------------------------------------------------------
        % Check the lower and upper limits on the event constraints in the current phase
        % ------------------------------------------------------------------- 
        if any(event_vector_max(:,:) - event_vector_min(:,:) < 0)
            error('Event constraint bounds inconsistant (i.e. max < min) in phase: %i',iphase)
        end
        if any(isnan(event_vector_min)) || any(isnan(event_vector_max))
            error('Event bounds NaN in phase: %i',iphase)
        end
    else
        event_vector_min = [];
        event_vector_max = [];
    end
    state_vector_min = state_matrix_min(:);
    state_vector_max = state_matrix_max(:);
    control_vector_min = control_matrix_min(:);
    control_vector_max = control_matrix_max(:);
    ode_vector_min = zeros((nodes(iphase)-1)*sizes(iphase,1),1);
    ode_vector_max = zeros((nodes(iphase)-1)*sizes(iphase,1),1);
    path_vector_min = path_matrix_min(:);
    path_vector_max = path_matrix_max(:);
    % ------------------------------------------------------------------------------------------------------
    % The cell array NLPLIMITS contains the lower and upper limits on the NLP variables in the current phase
    % ------------------------------------------------------------------------------------------------------
    nlplimits{iphase,1} = [state_vector_min; control_vector_min; t0_min; tf_min; parameter_min];
    nlplimits{iphase,2} = [state_vector_max; control_vector_max; t0_max; tf_max; parameter_max];
    nlplimits{iphase,3} = [ode_vector_min; path_vector_min; event_vector_min];
    nlplimits{iphase,4} = [ode_vector_max; path_vector_max; event_vector_max];
    variables(iphase) = length(nlplimits{iphase,1});
    constraints(iphase) = length(nlplimits{iphase,3});
    variable_indices{iphase} = variable_offset+1:variable_offset+variables(iphase);
    constraint_indices{iphase} = constraint_offset+1:constraint_offset+constraints(iphase);
    nstates = sizes(iphase,1);
    ncontrols = sizes(iphase,2);
    nparameters = sizes(iphase,3);
    state_indices = 1:(nodes(iphase))*nstates;
    control_indices = state_indices(end)+1:state_indices(end)+nodes(iphase)*ncontrols;
    if ~isempty(control_indices)
        t0_index = control_indices(end)+1;
    else
        t0_index = state_indices(end)+1;
    end
    tf_index = t0_index+1;
    parameter_indices = tf_index+1:tf_index+nparameters;
    indices(iphase).state = state_indices;
    indices(iphase).control = control_indices;
    indices(iphase).time = [t0_index; tf_index];
    indices(iphase).parameter = parameter_indices;
    variable_offset = variable_offset+variables(iphase);
    constraint_offset = constraint_offset+constraints(iphase);
end
numlinkpairs = length(linkages);
linkMinTot = [];
linkMaxTot = [];
for ipair=1:numlinkpairs
    % ------------------------------------------------------------%
    % Check the lower and upper limits on the linkage constraints %
    % ------------------------------------------------------------%
    if ~isfield(linkages(ipair),'min') || ~isfield(linkages(ipair),'max')
        error('Must Specify Both Lower and Upper Bounds on Linkage Constraints');
    else
        linkMin = linkages(ipair).min;
        linkMax = linkages(ipair).max;
        if ~isequal(size(linkMin),size(linkMax))
            error('Linkage Upper and Lower Bound Vector Must Be Same Size');
        end
        if ~isequal(size(linkMin,2), 1)
            errStr = 'Linkage Lower Bound Vector Must Be Column Vector: Number_Links x 1';
            error('%s, in Linkage Pair %i',errStr,ipair)
        end
        if ~isequal(size(linkMax,2), 1)
            errStr = 'Linkage Upper Lower Bound Vector Must Be Column Vector: Number_Links x 1';
            error('%s, in Linkage Pair %i',errStr,ipair)
        end
        if any(linkMin - linkMax > 0)
            error('Linkage Constraint Bounds Inconsistent (i.e. max < min) in Linkage Pair %i',ipair)
        end
        if any(isnan(linkMin)) || any(isnan(linkMax))
            error('Linkage bounds NaN in Linkage Pair %i',ipair)
        end
    end
    if ~isfield(linkages(ipair),'left') || ~isfield(linkages(ipair),'right')
        error('Must Specify Both Phases To be Linked');
    else
        leftPhase = linkages(ipair).left.phase;
        rightPhase = linkages(ipair).right.phase;
        if ~isequal(size(leftPhase),[1 1]) || ~isequal(size(rightPhase),[1 1])
            error('Left and Right Phase Numbers Must be Integers\n\t');
        end
    end
    if isequal(leftPhase,rightPhase) || (leftPhase <= 0) || (rightPhase > numphases) || (rightPhase <= 0) || (rightPhase > numphases) || ~isequal(leftPhase,round(leftPhase)) || ~isequal(rightPhase,round(rightPhase))
        error('Invalid Linkage phase numbers in connection: %i',ipair)
    end
    linkMinTot = [linkMinTot; linkMin];
    linkMaxTot = [linkMaxTot; linkMax];
end
numlinks = length(linkMinTot);
varbounds_min = vertcat(nlplimits{:,1});
varbounds_max = vertcat(nlplimits{:,2});
conbounds_min = vertcat(nlplimits{:,3});
conbounds_max = vertcat(nlplimits{:,4});
conbounds_min = [conbounds_min; linkMinTot];
conbounds_max = [conbounds_max; linkMaxTot];
setup.varbounds_min = varbounds_min;
setup.varbounds_max = varbounds_max;
setup.conbounds_min = conbounds_min;
setup.conbounds_max = conbounds_max;
setup.nlplimits = nlplimits;
setup.variables = variables;
setup.constraints = constraints;
setup.variable_indices = variable_indices;
setup.constraint_indices = constraint_indices;
setup.numphases = numphases;
setup.numnonlin = length(conbounds_min);
setup.numlinks  = numlinks;
setup.indices = indices;

% --------------------------------
% Get Bounds on Linear Constraints
% --------------------------------
numvars = length(varbounds_min);
Alinear = zeros(numphases+numlinkpairs,numvars);
Alinear = sparse(Alinear);
Alinmin = zeros(numphases+numlinkpairs,1);
Alinmax = zeros(numphases+numlinkpairs,1);
% ---------------------------------------------
% Part 1:  Monotonicity of Independent Variable
% ---------------------------------------------
for iphase=1:numphases
    nstates = sizes(iphase,1);
    ncontrols = sizes(iphase,2);
    if ~isequal(iphase,1)
        ishift = variable_indices{iphase-1}(end);
    else
        ishift =0;
    end
    t0_index = ishift+(nodes(iphase))*nstates+nodes(iphase)*ncontrols+1;
    tf_index = t0_index+1;
    Alinear(iphase,t0_index) = -1;
    Alinear(iphase,tf_index) = 1;
    if isfield(limits(iphase),'duration')
        % Check if only ONE of the lower and upper bounds on the phase duration are specified
        if (isfield(limits(iphase).duration,'min') && ~ isfield(limits(iphase).duration,'max')) || (~isfield(limits(iphase).duration,'min') && isfield(limits(iphase).duration,'max'))
            error('Must Specify Both Lower and Upper Bounds on Phase Duration');
        else 
            if isempty(limits(iphase).duration.min) && isempty(limits(iphase).duration.max)
                % Neither Minimum nor Maximum Duration Specified
                Alinmin(iphase) = 0;
                Alinmax(iphase) = Inf;
            elseif ~isempty(limits(iphase).duration.min) && isempty(limits(iphase).duration.max)
                % Minimum Duration Specified But Maximum Duration Unspecified
                if isequal(size(limits(iphase).duration.min),[1 1]) 
                    Alinmin(iphase) = limits(iphase).duration.min;
                    Alinmax(iphase) = Inf;
                end
            elseif isempty(limits(iphase).duration.min) && ~isempty(limits(iphase).duration.max)
                % Minimum Duration Unspecified But Maximum Duration Specified
                if isequal(size(limits(iphase).duration.max),[1 1])
                    Alinmin(iphase) = 0;
                    Alinmin(iphase) = limits(iphase).duration.max;
                end
            else
                % Both Minimum and Maximum Duration Specified
                if ~(isequal(size(limits(iphase).duration.min),[1 1]) && isequal(size(limits(iphase).duration.max),[1 1]))
                    errStr1 = 'Lower and Upper Bounds on Phase Duration in Phase ';
                    errStr2 = ' Must be Scalars';
                    error('%s, in Linkage Pair %i %s',errStr1,ipair,errStr2)
                else
                    Alinmin(iphase) = limits(iphase).duration.min;
                    Alinmax(iphase) = limits(iphase).duration.max;
                end
            end
        end
    else
        Alinmin(iphase) = 0;
        Alinmax(iphase) = Inf;
    end
end
istart = numphases;
% --------------------------------------
% Part 2:  Linkage of Time Across Phases
% --------------------------------------
for ipair=1:numlinkpairs
    left_phase = linkages(ipair).left.phase;
    right_phase = linkages(ipair).right.phase;
    nparameters_left = sizes(left_phase,3);
    nparameters_right = sizes(right_phase,3);
    tf_index_left = variable_indices{left_phase}(end-nparameters_left);
    t0_index_right = variable_indices{right_phase}(end-nparameters_right-1);
    Alinear(istart+ipair,tf_index_left) = -1;
    Alinear(istart+ipair,t0_index_right) = 1;
    Alinmin(istart+ipair) = 0;
    Alinmax(istart+ipair) = 0;
end
setup.numnonlin = setup.numnonlin;
setup.numvars   = numvars;
setup.numlin    = length(Alinmin);
setup.numlinkpairs = numlinkpairs;
setup.Alinear   = Alinear;
setup.Alinmin   = Alinmin;
setup.Alinmax   = Alinmax;
