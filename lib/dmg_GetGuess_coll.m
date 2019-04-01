function setup = gpopsGetGuess_coll(setup);
%------------------------------------------------------------------%
% Get the guess used by the NLP solver in a non-sequential         %
% multiple-phase optimal control problem.                          %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

guess = setup.guess;
numphases = setup.numphases;
if length(guess)<numphases
    error('Number of phases in Guess is less than number of phases in limits')
end
if length(guess)>numphases,
    guess = guess(1:numphases);
end;
sizes = setup.sizes;
nodes = setup.nodes;
init = cell(numphases,1);

for iphase=1:numphases;
    %--------------------------------------------%
    % Get the guess in each phase of the problem %
    %--------------------------------------------%
    nstates = sizes(iphase,1);
    ncontrols = sizes(iphase,2);
    nparameters = sizes(iphase,3);
    npaths = sizes(iphase,4);
    nevents = sizes(iphase,5);
    tinit    = guess(iphase).time;
    xinit    = guess(iphase).state;
    if isfield(guess(iphase),'control'),
        uinit    = guess(iphase).control;
    else
        uinit = [];
    end;
    if isfield(guess(iphase),'parameter'),
        pinit    = guess(iphase).parameter;
    else
        pinit = [];
    end;
    %-------------------------------%
    % Check guess for proper format %
    %-------------------------------%
    [length_guess col_t] = size(tinit);
    if col_t ~= 1
        error('Guess for time in phase %i must be a column vector',iphase)
    elseif length_guess < 2
        error('Guess in phase %i must have a least two points',iphase)
    elseif length_guess ~= length(unique(tinit))
        sprintf('WARNING: Guess for time in phase %i does not contain unique values',iphase)
    end
    [row_state col_state] = size(xinit);
    if row_state ~= length_guess
        error('Size of state does not match size of time in guess for phase %i',iphase) 
    elseif col_state ~= nstates
        error('Number of states in guess does not match limits in phase %i',iphase) 
    end
    [row_control col_control] = size(uinit);
    if row_control ~= 0 && row_control ~= length_guess
        error('Size of control does not match size of time in guess for phase %i',iphase) 
    elseif col_control ~= ncontrols
        error('Number of controls in guess does not match limits in phase %i',iphase) 
    end
    [row_param col_param] = size(pinit);
    if col_param ~= 0 && col_param ~= 1
        error('Guess for parameters in phase %i must be a column vector',iphase)
    elseif row_param ~= nparameters
        error('Number of parameters in guess does not match limits in phase %i',iphase) 
    end
    %---------------------------------------%
    % Check Cost function for proper format %
    %---------------------------------------%
    if ~isempty(setup.funcs.cost),
        if ~(isa(setup.funcs.cost,'char') || isa(setup.funcs.cost,'function_handle'))
            error('Invalid Cost function in setup.funcs.cost')
        end
        clear sol
        sol.initial.time = tinit(1);
        sol.initial.state = xinit(1,:).';
        sol.terminal.time = tinit(2);
        sol.terminal.state = xinit(end,:).';
        sol.time = tinit;
        sol.state = xinit;
        sol.control = uinit;
        sol.parameter = pinit;
        sol.phase = iphase;
        % try user cost function
        [Mayer, Lagrange] = feval(setup.funcs.cost,sol,setup);
        % check size of outputs
        if ~isscalar(Mayer)
            error('Cost function "%s" did not return scalar for Mayer cost using guess in phase %i',setup.funcs.cost,iphase)
        end
        [row_L col_L] = size(Lagrange);
        if row_L ~= length_guess || col_L ~= 1
            error('Cost function "%s" returned invalid size of column vector for Lagrange cost using guess in phase %i',setup.funcs.cost,iphase)
        end
    else
        error('Must Specify a Cost Function for Problem');
    end;
    %---------------------------------------------------------%
    % Check Differential-Algebraic function for proper format %
    %---------------------------------------------------------%
    if isfield(setup.funcs','dae'),
        if ~isempty(setup.funcs.dae),
            if ~(isa(setup.funcs.dae,'char') || isa(setup.funcs.dae,'function_handle'))
                error('Invalid Dae function in setup.funcs.dae')
            end;
        end
        clear sol
        sol.time      = tinit;
        sol.state     = xinit;
        sol.control   = uinit;
        sol.parameter = pinit; 
        sol.phase     = iphase;
        % try user dae function
        dae = feval(setup.funcs.dae,sol,setup);
        % check size of outputs
        [row_dae col_dae] = size(dae);
        if col_dae ~= (nstates+npaths)
            error('Dae function "%s" returned invalid number of columns using guess in phase %i',setup.funcs.dae,iphase) 
        elseif row_dae ~= length_guess
            error('Dae function "%s" returned invalid number of rows using guess in phase %i',setup.funcs.dae,iphase)
        end
    else 
        if (nstates>0),
            error('Must Specify a Differential-Algebraic Function When # of States is Nonzero');
        end;
    end;
    % ---------------------------------------%
    % Check Event function for proper format %
    % ---------------------------------------%
    if (nevents > 0) || isfield(setup.funcs,'event'),
        if ~isempty(setup.funcs.event),
            if ~(isa(setup.funcs.event,'char') || isa(setup.funcs.event,'function_handle'))
                error('Invalid Event function in setup.funcs.event')
            end
            clear sol
            sol.initial.time = tinit(1);
            sol.initial.state = xinit(1,:).';
            sol.terminal.time = tinit(end);
            sol.terminal.state = xinit(end,:).';
            sol.parameter = pinit;
            sol.phase = iphase;
            solTotal(iphase) = sol;
            % try user Event function
            event = feval(setup.funcs.event,sol,setup);
            % check size of outputs
            [row_event col_event] = size(event);
            if nevents > 0 && col_event ~= 1
                error('Event function "%s" must return column vector in phase %i',setup.funcs.event,iphase)
            elseif row_event ~= nevents
                error('Event function "%s" returned invalid number of events using guess in phase %i',setup.funcs.event,iphase)
            end
        else
            if (nevents>0),
                error('Must Specify Event Function When # of Events is Nonzero');
            end;
        end;
    end
    % ------------------%
    % Interpolate guess %
    % ------------------%
    t0init             = tinit(1);
    tfinit             = tinit(end);
    t = linspace(0,1,nodes(iphase));
    if abs(tfinit-t0init)<1e-6
        tfinit = tfinit+0.01;
        tinit = linspace(t0init,tfinit,length(tinit));
    end
    
    tinterp  = (tfinit-t0init)*t+t0init;
    
    if nstates>0,
        xinterp = interp1(tinit,xinit,tinterp,'spline');
    else
        xinterp = [];
    end;
    if ncontrols>0,
        
        uinterp = interp1(tinit,uinit,tinterp,'linear');
        
    else
        uinterp = [];
    end;
    init{iphase,1} = [xinterp(:); uinterp(:); t0init; tfinit; pinit];
end;
init_vector = vertcat(init{:,1});
setup.init_vector = init_vector;

%--------------------------------------------%
% Check Connection function for proper format %
%--------------------------------------------%
numlinks = setup.numlinks;
linkages = setup.linkages;
numlinkpairs = setup.numlinkpairs;
if numlinks > 0 
    if ~(ischar(setup.funcs.link) || isa(setup.funcs.link,'function_handle')),
        error('Invalid Linkage function in setup.funcs.link')
    end
    for ipair = 1:numlinkpairs;
        leftPhase = linkages(ipair).left.phase;
        rightPhase = linkages(ipair).right.phase;
        nlink = length(setup.linkages(ipair).min);
        clear sol
        sol.left.state       = guess(leftPhase).state(end,:).';
        sol.left.parameter   = guess(leftPhase).parameter;
        sol.left.time        = solTotal(leftPhase).terminal.time;
        sol.left.phase        = leftPhase;
        sol.right.state      = guess(rightPhase).state(1,:).';
        sol.right.time  =      solTotal(rightPhase).initial.time;
        sol.right.parameter  = guess(rightPhase).parameter;
        sol.right.phase       = rightPhase;
        %---------------------------%
        % try user linkage function %
        %---------------------------%
        link = feval(setup.funcs.link,sol,setup);
        % check size of outputs
        [row_con col_con] = size(link);
        if nlink > 0 && col_con ~= 1
            error('Phase-connect function "%s" must return column vector for connection %i',setup.funcs.link,ipair)
        elseif row_con ~= nlink
            error('Phase-connect function "%s" returned invalid number of connection constraints for connection %i'...
                ,setup.funcs.link,ipair)
        end
    end
else
    if isfield(setup.funcs,'link'),
        if ~isempty(setup.funcs.link),
            if ischar(setup.funcs.link) || isa(setup.funcs.link,'function_handle')
                error('Phase-connect function defined in setup.funcs.link with no connections')
            end
        end;
    end;
end
