function setup = gpopsNlp2oc(setup);
%------------------------------------------------------------------%
% Convert the NLP solution to trajectory format                    %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global extras funcs;

clear snoptcmex
limits = setup.limits;
funcs = setup.funcs;
ps = setup.ps;
result = setup.result;
variable_indices = setup.variable_indices;
constraint_indices = setup.constraint_indices;
sizes = setup.sizes;
nodes = setup.nodes;
numnonlin = setup.numnonlin;
numphases = setup.numphases;
numlinks = setup.numlinks;
x = result.x;
lambda  = result.Fmul(1:numnonlin-numlinks);
cost = 0;
varshift = 0;
conshift = 0;
solution = repmat(struct('phase',[],'time',[],'state',[],'control',[],...
    'parameter',[],'costate',[],'pathmult',[],'Hamiltonian',[],...
    'Mayer_cost',[],'Lagrange_cost',[]),1,numphases);
for i=1:numphases;
    %--------------------------------------------------------------------%
    % First construct the primal solution to the optimal control         %
    % problem (i.e., states, controls, and parameters)                   %
    %--------------------------------------------------------------------%
    xcurr = x(variable_indices{i});
    nstates = sizes(i,1);
    ncontrols = sizes(i,2);
    nparameters = sizes(i,3);
    npaths = sizes(i,4);
    state_indices = 1:(nodes(i)+2)*nstates;
    if ncontrols>0,
        control_indices = state_indices(end)+1:state_indices(end)+nodes(i)*ncontrols;
        t0_index = control_indices(end)+1;
    else
        control_indices = [];
        t0_index = state_indices(end)+1;
    end;
    tf_index = t0_index+1;   
    parameter_indices = tf_index+1:tf_index+nparameters;
    t0 = xcurr(t0_index);
    tf = xcurr(tf_index);
    state_vector = xcurr(state_indices);
    control_vector = xcurr(control_indices);
    tau_all = [-1; ps(i).Points; 1];
    t_all = (tf-t0)*(tau_all+1)/2+t0;
    t_gauss = t_all(2:end-1);
    if nstates>0,
        state_matrix = reshape(state_vector,nodes(i)+2,nstates);
    else
		state_matrix = [];
    end;
    if ncontrols>0,
        control_matrix = reshape(control_vector,nodes(i),ncontrols);
        control_t0 = zeros(1,ncontrols);
        control_tf = control_t0;
        if (tf-t0)>1e-13,
            for j=1:ncontrols
                control_t0(j) = interp1(t_gauss,control_matrix(:,j),t0,'spline','extrap');
                control_tf(j) = interp1(t_gauss,control_matrix(:,j),tf,'spline','extrap');
            end;
        else
            control_t0 = control_matrix(1,:);
            control_tf = control_matrix(end,:);
        end;
        control_matrixTotal = [control_t0; control_matrix; control_tf];
        control_matrixTotalExtrap = [control_t0; control_matrix; control_tf];
    else
        control_matrixTotal = [];
        control_matrixTotalExtrap = [];
    end;
    parameter = xcurr(parameter_indices);
    %------------------------------%
    % Next, construct the costates %
    %------------------------------%
    lamcurr = lambda(conshift+1:conshift+(nodes(i)+1)*nstates);
    lamcurr = reshape(lamcurr,nodes(i)+1,nstates);
    costatef = lamcurr(end,:);
    Inverse_Weight_Matrix = diag(1./ps(i).Weights);
    costate_gauss = Inverse_Weight_Matrix*lamcurr(1:end-1,:);
    DD = ps(i).D(:,1);
    DD = repmat(DD,1,nstates);
    costate0 = sum(-DD.*lamcurr(1:end-1,:),1);
    alpha0 = ps(i).Weights*ps(i).D(:,1);
    costate0 = costate0+(1+alpha0)*costatef;
    costate = [costate0; costate_gauss; costatef];
    %--------------------------------------------%
    % Solve for the initial and terminal control %
    %--------------------------------------------%
    if ncontrols>0,
        ss0 =      '---------------------------------------------------';
        ss=strcat(['BEGIN:  Computation of Endpoint Controls in Phase ',num2str(i)]);
        disp(' ');
        disp(' ');
        disp(ss0);
        disp(ss);
        disp(ss0);
        umin = limits(i).control.min;
        umax = limits(i).control.max;
        if npaths>0,
            pathmin = limits(i).path.min;
            pathmax = limits(i).path.max;
        else
            pathmin = [];
            pathmax = [];
        end;
        Flow = [-Inf; pathmin];
        Fupp = [ Inf; pathmax];
        extras.phase = i;
        extras.time = t_all(1);
        extras.state = state_matrix(1,:);
        extras.costate = costate0;
        extras.parameter = parameter;
        usrfun = 'gpopsEndPointControl';
        snscreen on
        uguess = control_t0.';
        snseti('Derivative Option',0);
        snprint('snoptmain0.out');
        snseti('Timing level',3);
        % ---------------------------------------%
        % Compute the Control at Initial Time    %
        % Using the Pontryagin Minimum Principle %
        % ---------------------------------------%
        m = length(Flow); n = length(uguess);
        umul   = zeros(n,1); ustate = zeros(n,1);
        Fmul   = zeros(m,1); Fstate = zeros(m,1);
        ObjAdd = 0; ObjRow = 1; solveopt = 1;
        Sjacobian = ones(m-1,n);
        Alinear = [];
        S_all = [ones(1,n); Sjacobian; zeros(size(Alinear))];
        [iGfun,jGvar]=find(S_all);
        iGfun = iGfun(:);
        jGvar = jGvar(:);
        A = []; iAfun = []; jAvar = [];
        [u0out,F0out,xmul0out,Fmul0out,inform0] = snoptcmex(solveopt,uguess,umin,umax,umul,ustate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar,usrfun);
        snprint off
        extras.time = t_all(end);
        extras.state = state_matrix(end,:);
        extras.costate = costatef;
        extras.parameter = parameter;
        funcs = setup.funcs;
        uguess = control_tf.';
        snseti('Derivative Option',0);
        snprint('snoptmainF.out');
        snseti('Timing level',3);
        % ---------------------------------------%
        % Compute the Control at Initial Time    %
        % Using the Pontryagin Minimum Principle %
        % ---------------------------------------%
        m = length(Flow); n = length(uguess);
        umul   = zeros(n,1); ustate = zeros(n,1);
        Fmul   = zeros(m,1); Fstate = zeros(m,1);
        ObjAdd = 0; ObjRow = 1; solveopt = 1;
        [uFout,FFout,umulFout,FmulFout,informF] = snoptcmex(solveopt,uguess,umin,umax,umul,ustate,Flow,Fupp,Fmul,Fstate,ObjAdd,ObjRow,A,iAfun,jAvar,iGfun,jGvar,usrfun);
        snprint off
        ss0 =      '------------------------------------------------';
        ss=strcat(['END: Computation of Endpoint Controls in Phase ',num2str(i)]);
        disp(' ');
        disp(' ');
        disp(ss0);
        disp(ss);
        disp(ss0);
        control_matrixTotal(1,:) = u0out;
        control_matrixTotal(end,:) = uFout;
    end;
    if npaths>0,
        pathmult = lambda(conshift+(nodes(i)+1)*nstates+1:conshift+(nodes(i)+1)*nstates+npaths*nodes(i));
        pathmult = reshape(pathmult,nodes(i),npaths);
        for j=1:npaths
            pathmult(:,j) = 2*pathmult(:,j)/(tf-t0)./setup.ps(i).Weights.';
        end;
        if ncontrols>0,
            pathmultTotal = [Fmul0out(2:end).'; pathmult; FmulFout(2:end).'];
        else
            pathmult0 = zeros(1,npaths);
            pathmultF = zeros(1,npaths);
            for k=1:npaths
                pathmult0(k) = interp1(t_all(2:end-1),pathmult(:,k),t_all(1),'spline','extrap');
                pathmultF(k) = interp1(t_all(2:end-1),pathmult(:,k),t_all(end),'spline','extrap');
            end;
            pathmultTotal = [pathmult0; pathmult; pathmultF];
        end;
    else
        pathmultTotal = [];
    end;    
    % ----------------------------------------------------------------------%
    % Put all of the information from the primal and dual solutions         %
    % into the appropriate location in the cell array SOLUTION              %
    % i = phase number                                                      %
    % solution(i).time: vector containing time                              %
    % solution(i).state: N+2 by nx matrix containing states                 %
    % solution(i).control: N+2 by nu matrix containing controls             %
    % solution(i).parameter: vector of length q containing static parameters%
    % solution(i).costate: N+2 by np matrix containing costates             %
    % solution(i).pathmult: N+2 by np matrix containing path multipliers    %
    % solution(i).Hamiltonian: vector of length N+2 containing Hamiltonian  %
    % solution(i).Mayer_cost: scalar containing the Mayer cost              %
    % solution(i).Lagrange_cost: scalar containing the Lagrange cost        %
    sol.initial.time = t_all(1);
    sol.initial.state = state_matrix(1,:).';
    sol.terminal.time = t_all(end);
    sol.terminal.state = state_matrix(end,:).';
    sol.time = t_all;
    sol.state = state_matrix;
%    sol.control = control_matrixTotal;
    sol.control = control_matrixTotalExtrap;
    sol.parameter = parameter;
    sol.phase = i;
    [Mayer,Lagrange] = feval(setup.funcs.cost,sol);
    soldae.time = t_all;
    soldae.state = state_matrix;
    soldae.control = control_matrixTotal;
    soldae.parameter = parameter;
    soldae.phase = i;
    dae = feval(setup.funcs.dae,soldae,setup);
    Hamiltonian = Lagrange+sum(costate.*dae(:,1:nstates),2);
    varshift = varshift+length(variable_indices{1,i});
    conshift = conshift+length(constraint_indices{1,i});
    Lagrange_Cost = (tf-t0)*setup.ps(i).Weights*Lagrange(2:end-1)/2;
    cost = cost + Mayer + Lagrange_Cost;
    solution(i).phase = soldae.phase;
    solution(i).time = soldae.time;
    solution(i).state = soldae.state;
    solution(i).control = control_matrixTotalExtrap;
    solution(i).control_pontryagin = soldae.control;
    solution(i).parameter = soldae.parameter;
    solution(i).costate = costate;
    solution(i).pathmult = pathmultTotal;
    solution(i).Hamiltonian = Hamiltonian;
    solution(i).Mayer_cost = Mayer;
    solution(i).Lagrange_cost = Lagrange_Cost;
end;
setup.solution = solution;
setup.cost     = cost;
