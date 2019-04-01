function[Cost,G]=objective(x,mysetup)
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
Cost = 0;
%
[ncon, nvar] = size(mysetup.sparsity_all);
G = zeros(1,nvar);
%
x = x./mysetup.column_scales;
    
for i=mysetup.numphases
    
    xcurr = x(mysetup.variable_indices{i});
    nstates = mysetup.sizes(i,1);
    ncontrols = mysetup.sizes(i,2);
    nparameters = mysetup.sizes(i,3);
    npaths = mysetup.sizes(i,4);
    nevents = mysetup.sizes(i,5);
    t0 = xcurr(mysetup.indices(i).time(1));
    tf = xcurr(mysetup.indices(i).time(2));
    state_vector = xcurr(mysetup.indices(i).state);
    control_vector = xcurr(mysetup.indices(i).control);
    tau_all = [-1; mysetup.ps(i).Points; 1];
    t_all = (tf-t0)*(tau_all+1)/2 + t0;
    t_gauss = t_all(2:end-1);
    state_matrix = reshape(state_vector,mysetup.nodes(i)+2,nstates);
    state_gauss = state_matrix(2:end-1,:);
    x0 = state_matrix(1,:);
    xf = state_matrix(end,:);
    control_gauss = reshape(control_vector,mysetup.nodes(i),ncontrols);
    parameters = xcurr(mysetup.indices(i).parameter);
    
    %---------------%
    % Get user Cost %
    %---------------%
    solcost.initial.time = t0;
    solcost.initial.state = x0.';
    solcost.terminal.time = tf;
    solcost.terminal.state = xf.';
    solcost.time = t_gauss;
    solcost.state = state_gauss;
    solcost.control = control_gauss;
    solcost.parameter = parameters;
    solcost.phase = i;
  
    if isequal(lower(mysetup.derivatives),'analytic')
    [Mayer,Lagrange,DMayer,DLagrange] = feval(mysetup.funcs.cost,solcost);
    else
    [Mayer,Lagrange] = feval(mysetup.funcs.cost,solcost,mysetup);
    end
    %
   integrand = (tf-t0)*mysetup.ps(i).Weights*Lagrange/2;
    
   Cost = Cost + Mayer + integrand;
    
   
   if isequal(lower(mysetup.derivatives),'analytic')
       %
       nnodes = mysetup.nodes(i);
        %----------------------%
        % Get Cost derivatives %
        %----------------------%
        Jcost = zeros(1,(nstates+ncontrols)*nnodes+2*nstates+nparameters+2);
        % dCost/dx
        for jj = 1:nstates
            col0 = nnodes*(jj-1)+2*jj-1;
            cols = nnodes*(jj-1)+2*jj:nnodes*(jj)+2*jj-1;
            colF = nnodes*(jj)+2*jj;
            Jcost(1,col0) = DMayer(1,jj);
            Jcost(1,cols) = (tf-t0)*mysetup.ps(i).Weights.*DLagrange(:,jj).'/2;
            Jcost(1,colF) = DMayer(1,jj + nstates+1);
        end
        % dCost/du
        for jj = nstates+1:nstates+ncontrols
            cols = (nnodes+2)*(nstates) + (nnodes*(jj-1-nstates)+1:nnodes*(jj-nstates));
            Jcost(1,cols) = (tf-t0)*mysetup.ps(i).Weights.*DLagrange(:,jj).'/2;
        end
        % dCost/dparam
        for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
            cols = nnodes*(nstates+ncontrols)+ 2*nstates + jj -(nstates+ncontrols);
            Jcost(1,cols) = DMayer(1,2*(nstates+1)+jj-(nstates+ncontrols)) + (tf-t0)*mysetup.ps(i).Weights.*DLagrange(:,jj).'/2;
        end
        jj = nstates + ncontrols + nparameters;
        % dCost/dt0
        cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+1;
        Jcost(1,cols) = DMayer(1,nstates+1) - mysetup.ps(i).Weights*Lagrange/2 + (tf-t0)/2*mysetup.ps(i).Weights*diag(DLagrange(:,jj+1))*(-mysetup.ps(i).Points/2+1/2);
        % dCost/dtf
        cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+2;
        Jcost(1,cols) = DMayer(1,2*nstates+2) + mysetup.ps(i).Weights*Lagrange/2 + (tf-t0)/2*mysetup.ps(i).Weights*diag(DLagrange(:,jj+1))*(mysetup.ps(i).Points/2+1/2); 
        % Map cost derivatives to sparsity pattern
        G(1,mysetup.variable_indices{i}) = Jcost;
        
   end
        
end
    
    
    
    
    
    
  