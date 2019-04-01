function[C,J]=dmg_constraints(x,mysetup)
%-----------------------------------------------------------------  -%
% Compute the nonlinear constraints function for IPOPT               %
%------------------------------------------------------------------  %
[ncon, nvar] = size(mysetup.sparsity_all);
J = zeros(ncon,nvar);

%
Cons = cell(mysetup.numphases,1);
%
for i=1:mysetup.numphases
    
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
    t_all = (tf-t0)*(tau_all+1)/2+t0;
    t_gauss = t_all(2:end-1);
    state_matrix = reshape(state_vector,mysetup.nodes(i)+2,nstates);
    state_gauss = state_matrix(2:end-1,:);
    x0 = state_matrix(1,:);
    xf = state_matrix(end,:);
    control_gauss = reshape(control_vector,mysetup.nodes(i),ncontrols);
    parameters = xcurr(mysetup.indices(i).parameter);
    %--------------%
    % Get user DAE %
    %--------------%
    sol.time = t_gauss;
    sol.state = state_gauss;
    sol.control = control_gauss;
    sol.parameter = parameters;
    sol.phase = i;
    if isequal(lower(mysetup.derivatives),'analytic')
        [dae_out,Ddae_out] = feval(mysetup.funcs.dae,sol,mysetup);
    else
        [dae_out] = feval(mysetup.funcs.dae,sol,mysetup);
    end
    solevents.initial.time = t0;
    solevents.initial.state = x0.';
    solevents.terminal.time = tf;
    solevents.terminal.state = xf.';
    solevents.parameter = parameters;
    solevents.phase = i;
    %-----------------%
    % Get constraints %
    %-----------------%
    odelhs  = mysetup.ps(i).Ddiag*state_matrix(1:mysetup.nodes(i)+1,:);
    oderhs  = (tf-t0)*dae_out(:,1:nstates)/2;
    paths   = dae_out(:,nstates+1:end);
    ode_defects = odelhs-oderhs;
    quad_defects = zeros(size(xf));
    defects = [ode_defects; quad_defects];
    if nevents>0
        if isequal(lower(mysetup.derivatives),'analytic')
            [events, Devents] = feval(mysetup.funcs.event,solevents,mysetup);
        else
            [events] = feval(mysetup.funcs.event,solevents,mysetup);
        end
    else
        events = [];
    end
    
    Cons{i,1} = [defects(:); paths(:); events];
    
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
    
    solTotal(i) = solcost;
    
    if isequal(lower(mysetup.derivatives),'analytic')
        %--------------------------------%
        % Get derivatives of constraints %
        %--------------------------------%
        nnodes = mysetup.nodes(i);
        Jcon = spalloc((nstates+npaths)*nnodes+nevents+nstates,...
            (nstates+ncontrols)*nnodes+2*nstates+nparameters+2,...
            (nstates+npaths)*(nstates+ncontrols+nparameters+2)+(nstates*2+nparameters)*nevents);
        for ii = 1:nstates
            daerows = nnodes*(ii-1)+1:nnodes*(ii);
            rows = nnodes*(ii-1)+ii:nnodes*(ii)+ii-1;
            % df/dx
            for jj = 1:nstates
                cols = nnodes*(jj-1)+2*jj:nnodes*(jj)+2*jj-1;
                if ii == jj
                    Jcon(rows,cols) = mysetup.ps(i).Ddiag(:,2:end) - (tf-t0)/2*diag(Ddae_out(daerows,jj));
                else
                    Jcon(rows,cols) = - (tf-t0)/2*diag(Ddae_out(daerows,jj));
                end
            end
            % df/du
            for jj = nstates+1:nstates+ncontrols
                cols = (nnodes+2)*(nstates) + (nnodes*(jj-1-nstates)+1:nnodes*(jj-nstates));
                Jcon(rows,cols) = - (tf-t0)/2*diag(Ddae_out(daerows,jj));
            end
            % df/dparam
            for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
                cols = nnodes*(nstates+ncontrols)+ 2*nstates + jj -(nstates+ncontrols);
                Jcon(rows,cols) = - (tf-t0)/2*Ddae_out(daerows,jj);
            end
            jj = nstates + ncontrols + nparameters;
            % df/dto
            cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+1;
            Jcon(rows,cols) = 1/2 * dae_out(:,ii) - (tf-t0)/2*Ddae_out(daerows,jj+1).*(-mysetup.ps(i).Points/2+1/2);
            % df/dtf
            cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+2;
            Jcon(rows,cols) = -1/2 * dae_out(:,ii) - (tf-t0)/2*Ddae_out(daerows,jj+1).*(mysetup.ps(i).Points/2+1/2);
        end
        for ii = (nstates + 1):(nstates+npaths)
            daerows = nnodes*(ii-1)+1:nnodes*(ii);
            rows = nnodes*(ii-1)+nstates+1:nnodes*(ii)+nstates;
            % dc/dx
            for jj = 1:nstates
                cols = nnodes*(jj-1)+2*jj:nnodes*(jj)+2*jj-1;
                Jcon(rows,cols) = diag(Ddae_out(daerows,jj));
            end
            % dc/du
            for jj = nstates+1:nstates+ncontrols
                cols = (nnodes+2)*(nstates) + (nnodes*(jj-1-nstates)+1:nnodes*(jj-nstates));
                Jcon(rows,cols) = diag(Ddae_out(daerows,jj));
            end
            % dc/dparam
            for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
                cols = nnodes*(nstates+ncontrols)+ 2*nstates + jj -(nstates+ncontrols);
                Jcon(rows,cols) = Ddae_out(daerows,jj);
            end
            jj = nstates + ncontrols + nparameters;
            % dc/dto
            cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+1;
            Jcon(rows,cols) = Ddae_out(daerows,jj+1).*(-mysetup.ps(i).Points/2+1/2);
            % dc/dtf
            cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+2;
            Jcon(rows,cols) = Ddae_out(daerows,jj+1).*(mysetup.ps(i).Points/2+1/2);
        end
        for ii = 1:nevents
            rows = nnodes*(npaths+nstates)+nstates + ii;
            % dEvent/dx
            for jj = 1:nstates
                col0 = nnodes*(jj-1)+2*jj-1;
                colF = nnodes*(jj)+2*jj;
                Jcon(rows,col0) = Devents(ii,jj);
                Jcon(rows,colF) = Devents(ii,jj + nstates+1);
            end
            % dEvent/dparam
            for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
                cols = nnodes*(nstates+ncontrols)+ 2*nstates + jj -(nstates+ncontrols);
                Jcon(rows,cols) = Devents(1,2*(nstates+1)+jj-(nstates+ncontrols));
            end
            % dEvent/dt0
            cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+1;
            Jcon(rows,cols) = Devents(1,nstates+1);
            % dEvent/dtf
            cols = nnodes*(nstates+ncontrols)+ 2*nstates +nparameters+2;
            Jcon(rows,cols) = Devents(1,2*nstates+2);
        end
        % Map constraint derivatives to sparsity pattern
        J(mysetup.constraint_indices{i}+1,mysetup.variable_indices{i}) = Jcon;
    end
    
end
%
Constraints = vertcat(Cons{:,1});
linkages = mysetup.linkages;
if ~isempty(mysetup.linkages),
    link_out = cell(mysetup.numlinkpairs,1);
    link_row = length(Constraints) + 1;
    for ipair=1:mysetup.numlinkpairs;
        nlinks = length(linkages(ipair).min);
        left_phase  = linkages(ipair).left.phase;
        right_phase = linkages(ipair).right.phase;
        xf_left = solTotal(left_phase).terminal.state;
        p_left  = solTotal(left_phase).parameter;
        x0_right = solTotal(right_phase).initial.state;
        p_right  = solTotal(right_phase).parameter;
        %----------------------------%
        % Get Connection constraints %
        %----------------------------%
        sollink.left.state = xf_left;
        sollink.left.parameter = p_left;
        sollink.left.phase = left_phase;
        sollink.left.time  = solTotal(left_phase).time(end);
        sollink.right.state = x0_right;
        sollink.right.parameter = p_right;
        sollink.right.phase = right_phase;
        sollink.right.time  = solTotal(right_phase).time(1);
        if ~isequal(lower(mysetup.derivatives),'analytic')
            [link_out{ipair,1}] = feval(mysetup.funcs.link,sollink,mysetup);
        else
            [link_out{ipair,1} Dlink_out] = feval(mysetup.funcs.link,sollink);
            %----------------------------%
            % Get Connection derivatives %
            %----------------------------%
            link_row = link_row(end)+ (1:nlinks);
            [rows,cols] = find(mysetup.sparsity_all(link_row,:));
            J(link_row,1:max(cols)) = sparse(rows,cols,Dlink_out(:));
        end
    end
    Clink = vertcat(link_out{:,1});
    
    Constraints = [Constraints; Clink];
end

C = [Constraints; mysetup.initlincons];
J = J(2:end,:);







