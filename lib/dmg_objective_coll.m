function[Cost,G]=objective_coll(x,mysetup)
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

Cost = 0;
%
[ncon, nvar] = size(mysetup.Alinear_augmented);
G = zeros(1,nvar);
%
x = x./mysetup.column_scales;
%
for iphase=1:mysetup.numphases;
    
    xcurr = x(mysetup.variable_indices{iphase});
    nstates = mysetup.sizes(iphase,1);
    ncontrols = mysetup.sizes(iphase,2);
    nparameters = mysetup.sizes(iphase,3);
    npaths = mysetup.sizes(iphase,4);
    nevents = mysetup.sizes(iphase,5);
    t0 = xcurr(mysetup.indices(iphase).time(1));
    tf = xcurr(mysetup.indices(iphase).time(2));
    state_vector = xcurr(mysetup.indices(iphase).state);
    control_vector = xcurr(mysetup.indices(iphase).control);
    control_vector = reshape(control_vector,mysetup.nodes(iphase),ncontrols);
    tc= linspace(1/(2*(mysetup.nodes(iphase)-1)),1-1/(2*(mysetup.nodes(iphase)-1)),mysetup.nodes(iphase)-1);
    t = (tf-t0)*linspace(0,1,mysetup.nodes(iphase))+t0;
    state_matrix = reshape(state_vector,mysetup.nodes(iphase),nstates);
    x0 = state_matrix(1,:);
    xf = state_matrix(end,:);
    parameters = xcurr(mysetup.indices(iphase).parameter);
    step = (tf-t0)/(mysetup.nodes(iphase)-1);
    n = mysetup.nodes(iphase);
    %---------------%
    % Get user Cost %
    %---------------%
    solcost.initial.time = t0;
    solcost.initial.state = x0.';
    solcost.terminal.time = tf;
    solcost.terminal.state = xf.';
    solcost.time = t';
    solcost.state = state_matrix;
    solcost.control = control_vector;
    solcost.parameter = parameters;
    solcost.phase = iphase;
    
    if isequal(lower(mysetup.derivatives),'analytic')
        [Mayer,Lagrange,DMayer,DLagrange] = feval(mysetup.funcs.cost,solcost);
    else
        [Mayer,Lagrange] = feval(mysetup.funcs.cost,solcost,mysetup);
    end
    
    integrand = 0;
    
    %if not(Lagrange == 0)
        %---------------------------------------%
        % Get the Lagrange Cost at the mid point
        %---------------------------------------%
        
        t_c= (tf-t0)*linspace(1/(2*(mysetup.nodes(iphase)-1)),1-1/(2*(mysetup.nodes(iphase)-1)),mysetup.nodes(iphase)-1)+t0;
        h=(tf-t0)/(mysetup.nodes(iphase)-1);
        sol.time = t';
        sol.state = state_matrix;
        sol.control = control_vector;
        sol.parameter = parameters;
        sol.phase = iphase;
        
        [dae_out_t] = feval(mysetup.funcs.dae,sol,mysetup);
        
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
        solcost.parameter = parameters;
        solcost.phase = iphase;
        
        if isequal(lower(mysetup.derivatives),'analytic')
            [Mayer_tc,Lagrange_tc,DMayer_tc,DLagrange_tc] = feval(mysetup.funcs.cost,solcost);
        else
            [Mayer_tc,Lagrange_tc] = feval(mysetup.funcs.cost,solcost,mysetup);
        end
        
        integrand = sum(step/6 * ( Lagrange(1:end-1)+Lagrange(2:end)+4*Lagrange_tc));
        
    %end
    
    Cost = Cost + Mayer + integrand;
    
    if isequal(lower(mysetup.derivatives),'analytic')
        %
        nnodes = mysetup.nodes(iphase);
        %----------------------%
        % Get Cost derivatives %
        %----------------------%
        Jcost = zeros(1,(nstates+ncontrols)*nnodes+nparameters+2);
        % dCost/dx
        % NO ESTOY SEGURO DE SI ESTA BIEN, HABRÍA QUE COMPARAR
        for jj = 1:nstates
            col0 = nnodes*(jj-1)+1;
            colF = nnodes*(jj);
            cols = nnodes*(jj-1)+1:nnodes*(jj);
            %
            Jcost(1,col0) = DMayer(1,jj);
            Jcost(1,colF) = DMayer(1,jj + nstates +1);
            %
            Jcost(1,cols) = Jcost(1,cols) + sum(step/6 * ( DLagrange(1:end-1,jj)+DLagrange(2:end,jj)+4*DLagrange_tc(:,jj)));
        end
        % dCost/du
        for jj = nstates+1:nstates+ncontrols
            cols = (nnodes)*(nstates) + (nnodes*(jj-1-nstates)+1:nnodes*(jj-nstates));
            Jcost(1,cols) = sum(step/6 * ( DLagrange(1:end-1,jj)+DLagrange(2:end,jj)+4*DLagrange_tc(:,jj)));
        end
        % dCost/dparam
        for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
            cols = nnodes*(nstates+ncontrols)+ 2*nstates + jj -(nstates+ncontrols);
            Jcost(1,cols) = DMayer(1,2*(nstates+1)+jj-(nstates+ncontrols)) + sum(step/6 * ( DLagrange(1:end-1,jj)+DLagrange(2:end,jj)+4*DLagrange_tc(:,jj)));
        end
        jj = nstates + ncontrols + nparameters;
        % dCost/dt0
        cols = nnodes*(nstates+ncontrols)+nparameters+1;
        Jcost(1,cols) = DMayer(1,nstates+1);% - sum(1/6 * ( Lagrange(1:end-1)+Lagrange(2:end)+4*Lagrange_tc)) + (tf-t0)/2*diag(DLagrange(:,jj+1))*(-mysetup.ps(iphase).Points/2+1/2);
        % dCost/dtf
        cols = nnodes*(nstates+ncontrols)+nparameters+2;
        Jcost(1,cols) = DMayer(1,2*nstates+2);% + sum(1/6 * ( Lagrange(1:end-1)+Lagrange(2:end)+4*Lagrange_tc)) + (tf-t0)/2*diag(DLagrange(:,jj+1))*(mysetup.ps(iphase).Points/2+1/2);
        % Map cost derivatives to sparsity pattern
        G(1,mysetup.variable_indices{iphase}) = Jcost;
        
    end
    
end
    
    
  