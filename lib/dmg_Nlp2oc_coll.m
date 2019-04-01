function setup = gpopsNlp2oc_coll(setup);
%------------------------------------------------------------------%
% Convert the NLP solution to trajectory format                    %
%------------------------------------------------------------------%
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

global extras funcs;

clear snoptcmex

funcs = setup.funcs;
result = setup.result;
variable_indices = setup.variable_indices;
sizes = setup.sizes;
nodes = setup.nodes;
numphases = setup.numphases;

x = result.x;
cost = 0;

solution = repmat(struct('phase',[],'time',[],'state',[],'control',[],...
    'parameter',[],'Mayer_cost',[],'Lagrange_cost',[]),1,numphases);

for i=1:numphases;
    %--------------------------------------------------------------------%
    % First construct the primal solution to the optimal control         %
    % problem (i.e., states, controls, and parameters)                   %
    %--------------------------------------------------------------------%
    xcurr = x(variable_indices{i});
    nstates = sizes(i,1);
    ncontrols = sizes(i,2);
    nparameters = sizes(i,3);
    state_indices = 1:(nodes(i))*nstates;
        
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
    t_all = (tf-t0)*linspace(0,1,setup.nodes(i))+t0;
    step = (tf-t0)/(setup.nodes(i)-1);
    
    if nstates>0,
        state_matrix = reshape(state_vector,nodes(i),nstates);
    else
		state_matrix = [];
    end;
    
    if ncontrols>0,
        control_matrix = reshape(control_vector,nodes(i),ncontrols);
        control_matrixTotalExtrap = control_matrix;
    else
        control_matrixTotalExtrap = [];
    end;
    
    parameter = xcurr(parameter_indices);
     
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
    

    
%-------Obtain Cost --------
  
    solcost.initial.time = t_all;
    solcost.initial.state = state_matrix(1,:).';
    solcost.terminal.time = t_all(end);
    solcost.terminal.state = state_matrix(end,:).';
    solcost.time = t_all;
    solcost.state = state_matrix;
    solcost.control = control_matrixTotalExtrap;
    solcost.parameter = parameter;
    solcost.phase = i;
    
    [Mayer,Lagrange] = feval(setup.funcs.cost,solcost,setup);
    
    integrand = 0;
    
    if not(Lagrange == 0)
    %---------------------------------------%
    % Get the Lagrange Cost at the mid point
    %---------------------------------------%
    
    t_c= (tf-t0)*linspace(1/(2*(setup.nodes(i)-1)),1-1/(2*(setup.nodes(i)-1)),setup.nodes(i)-1)+t0;
    h=(tf-t0)/(setup.nodes(i)-1);
    sol.time = t_all;
    sol.state = state_matrix;
    sol.control = control_matrixTotalExtrap;
    sol.parameter = parameter;
    sol.phase = i;
    
    [dae_out_t] = feval(setup.funcs.dae,sol,setup);
    
    %-----------------%
    % Get constraints %
    %-----------------%
    
    f = dae_out_t(:,1:nstates);
   
    y1 = state_matrix(1:end-1,:);
    y2 = state_matrix(2:end,:);
    f1 = f(1:end-1,:);
    f2 = f(2:end,:);
    
    state_matrix_tc = 0.5*(y1+y2) +h/8*(f1-f2);
    
    %linear interpolation for the control
    
    u1=control_vector(1:end-1,:);
    u2=control_vector(2:end,:);
    ulinear = 0.5.*(u2+u1);
    control_vector_tc = ulinear;
  
    
    solcost.time = t_c';
    solcost.state = state_matrix_tc;
    solcost.control = control_vector_tc;
    solcost.parameter = parameter;
    solcost.phase = i;
    
    [Mayer_tc,Lagrange_tc] = feval(setup.funcs.cost,solcost,setup);
        
    integrand = sum(step/6 * ( Lagrange(1:end-1)+Lagrange(2:end)+4*Lagrange_tc));
    
    end
    
    cost = cost + Mayer + integrand;
    
%--------------------------------------    
    
    
    soldae.time = t_all;
    soldae.state = state_matrix;
    soldae.parameter = parameter;
    soldae.phase = i;

  
    solution(i).phase = soldae.phase;
    solution(i).time = soldae.time;
    solution(i).state = soldae.state;
    solution(i).control = control_matrixTotalExtrap;
    solution(i).parameter = soldae.parameter;
    solution(i).Mayer_cost = Mayer;
    solution(i).Lagrange_cost = integrand;
    
end;
setup.solution = solution;
setup.cost     = cost;
