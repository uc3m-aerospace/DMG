function[C,J]=dmg_constraints_coll(x,mysetup)

%------------------------------------------------------------------%
%Compute the linear and nonlinear constraints and objective function%
%------------------------------------------------------------------%

Cons = cell(mysetup.numphases,1);

if nargout == 2
    [ncon, nvar] = size(mysetup.Alinear_augmented);
    J = zeros(ncon,nvar);
end

for i=1:mysetup.numphases
    %
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
    control_vector= reshape(control_vector,mysetup.nodes(i),ncontrols);
    t_c= (tf-t0)*linspace(1/(2*(mysetup.nodes(i)-1)),1-1/(2*(mysetup.nodes(i)-1)),mysetup.nodes(i)-1)+t0;
    t = (tf-t0)*linspace(0,1,mysetup.nodes(i))+t0;
    state_matrix = reshape(state_vector,mysetup.nodes(i),nstates);
    x0 = state_matrix(1,:);
    xf = state_matrix(end,:);
    parameters = xcurr(mysetup.indices(i).parameter); 
    %--------------%
    % Get user DAE %
    %--------------%
    h=(tf-t0)/(mysetup.nodes(i)-1);
    sol.time = t';
    sol.state = state_matrix;
    sol.control = control_vector;
    sol.parameter = parameters;
    sol.phase = i;
    %
    if isequal(lower(mysetup.derivatives),'analytic')
        [dae_out,Ddae_out] = feval(mysetup.funcs.dae,sol,mysetup);
    else
        [dae_out] = feval(mysetup.funcs.dae,sol,mysetup);
    end   
    %-----------------%
    % Get constraints %
    %-----------------%  
    f = dae_out(:,1:nstates);
    paths   = dae_out(:,nstates+1:end);
    %
    y1 = state_matrix(1:end-1,:);
    y2 = state_matrix(2:end,:);
    f1 = f(1:end-1,:);
    f2 = f(2:end,:);
    %
    state_matrix_tc = 0.5*(y1+y2) +h/8*(f1-f2);
    %
    %linear interpolation for the control
    %
    u1=control_vector(1:end-1,:);
    u2=control_vector(2:end,:);
    ulinear = 0.5.*(u2+u1);
    ulinear = u1;
    control_vector_tc = ulinear;
    %
    sol.time = t_c';
    sol.state = state_matrix_tc;
    sol.control = control_vector_tc;
    sol.parameter = parameters;
    sol.phase = i;
    %
    if isequal(lower(mysetup.derivatives),'analytic')
        [dae_out_tc,Ddae_out_tc] = feval(mysetup.funcs.dae,sol,mysetup);
    else
        [dae_out_tc] = feval(mysetup.funcs.dae,sol,mysetup);
    end
    %
    f_c = dae_out_tc(:,1:nstates);
    %
    defects = y1-y2 + h/6*(f1+f2+4*f_c);
    %
    solevents.initial.time = t0;
    solevents.initial.state = x0.';
    solevents.terminal.time = tf;
    solevents.terminal.state = xf.';
    solevents.parameter = parameters;
    solevents.phase = i;
    solTotal(i) = solevents;
    %
    if nevents>0
        if isequal(lower(mysetup.derivatives),'analytic')
            [events, Devents] = feval(mysetup.funcs.event,solevents,mysetup);
        else
            [events] = feval(mysetup.funcs.event,solevents,mysetup);
        end
    else
        events = [];
    end
    %
    Cons{i,1} = [defects(:); paths(:); events];
    %

if isequal(lower(mysetup.derivatives),'analytic')
        %--------------------------------%
        % Get derivatives of constraints %
        %--------------------------------%
        %
        nnodes = mysetup.nodes(i);
        Jcon = zeros((nstates+npaths)*(nnodes-1)+nevents,...
            (nstates+ncontrols)*nnodes+nparameters+2);
        %
        for ii = 1:nstates
            % df/dx
            for jj = 1:nstates
                %
                daerows   = nnodes*(ii-1)+1:nnodes*(ii);
                daerowstc = nnodes*(ii-1)+2-ii:nnodes*(ii)-ii;
                rows      = nnodes*(ii-1)+2-ii:nnodes*(ii)-ii;
                %
                cols    = nnodes*(jj-1)+1:nnodes*(jj);
                %
                for kk = 1:nnodes-1
                    %
                    daerowsk = kk:nnodes:nnodes*nstates-1;
                    %
                    sum1 = 0;
                    sum2 = 0;
                    %
                    for qq = 1:nstates
                        %
                        if jj==qq
                            %
                            sum1 = sum1 + ( h/8 * Ddae_out(daerowsk(qq),jj)   + 0.5) .* Ddae_out_tc(daerowstc(kk),qq);
                            sum2 = sum2 + (-h/8 * Ddae_out(daerowsk(qq)+1,jj) + 0.5) .* Ddae_out_tc(daerowstc(kk),qq);
                            %
                        else
                            %
                            sum1 = sum1 + ( h/8*Ddae_out(daerowsk(qq),jj)   .* Ddae_out_tc(daerowstc(kk),qq));
                            sum2 = sum2 + (-h/8*Ddae_out(daerowsk(qq)+1,jj) .* Ddae_out_tc(daerowstc(kk),qq));
                            %
                        end
                        %
                    end
                    %
                    if ii==jj
                        Jcon(rows(kk),cols(kk))   =  1 + h/6 * (Ddae_out(daerows(kk),jj)   + 4 * (sum1) ) ;
                        Jcon(rows(kk),cols(kk+1)) = -1 + h/6 * (Ddae_out(daerows(kk+1),jj) + 4 * (sum2) ) ;
                    else
                        %
                        Jcon(rows(kk),cols(kk))   =  h/6 * ( Ddae_out(daerows(kk),jj)   + 4 * (sum1) ) ;
                        Jcon(rows(kk),cols(kk+1)) =  h/6 * ( Ddae_out(daerows(kk+1),jj) + 4 * (sum2) ) ;
                    end
                    %
                end
                
            end
            % df/du
            for jj = nstates+1:nstates+ncontrols
                cols = (nnodes)*(nstates) + (nnodes*(jj-1-nstates)+1:nnodes*(jj-nstates));
                for kk = 1:nnodes-1
                    %
                    daerowsk = kk:nnodes:nnodes*nstates-1;
                    %
                    sum1 = 0;
                    sum2 = 0;
                    %
                    for qq = 1:nstates
                        %
                        sum1 = sum1 + h/8 * Ddae_out(daerowsk(qq),jj)   * Ddae_out_tc(daerowstc(kk),qq);
                        sum2 = sum2 - h/8 * Ddae_out(daerowsk(qq)+1,jj) * Ddae_out_tc(daerowstc(kk),qq);
                        %
                    end
                    % Used because of the linear interpolation between
                    % nodes
                    sum1 = sum1 + 0.5 * Ddae_out_tc(daerowstc(kk),jj);
                    sum2 = sum2 + 0.5 * Ddae_out_tc(daerowstc(kk),jj);
                    %
                    Jcon(rows(kk),cols(kk))   =  h/6 * ( Ddae_out(daerows(kk),jj)   + 4 * (sum1)) ;
                    Jcon(rows(kk),cols(kk+1)) =  h/6 * ( Ddae_out(daerows(kk+1),jj) + 4 * (sum2)) ;
                end
                %
            end
            % df/dparam
            for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
                cols = nnodes*(nstates+ncontrols) + jj -(nstates+ncontrols);
                for kk = 1:nnodes-1
                    %
                    daerowsk = kk:nnodes:nnodes*nstates-1;
                    %
                    sum1 = 0;
                    sum2 = 0;
                    %
                    for qq = 1:nstates
                        %
                        sum1 = sum1 + h/8*Ddae_out(daerowsk(qq),jj)   * Ddae_out_tc(daerowstc(kk),qq);
                        sum2 = sum2 - h/8*Ddae_out(daerowsk(qq)+1,jj) * Ddae_out_tc(daerowstc(kk),qq);
                        %
                    end
                    %
                    Jcon(rows(kk),cols(kk))   =  h/6 * ( Ddae_out(daerows(kk),jj)   + 4 * (sum1)) ;
                    Jcon(rows(kk),cols(kk+1)) =  h/6 * ( Ddae_out(daerows(kk+1),jj) + 4 * (sum2)) ;
                end
            end
            %
            % ESTO PREVISIBLEMENTE ESTE MAL (SOLO BIEN SI LAS DERIVADAS NO DEPENDEN DEL TIEMPO)
            %
             % df/dto
             cols = nnodes*(nstates+ncontrols) + nparameters + 1;
             Jcon(rows,cols) = -1/((nnodes-1)*6)*(f1(:,ii)+f2(:,ii)+4*f_c(:,ii)) ;
             % df/dtf
             cols = nnodes*(nstates+ncontrols) + nparameters + 2;
             Jcon(rows,cols) = +1/((nnodes-1)*6)*(f1(:,ii)+f2(:,ii)+4*f_c(:,ii)) ;   
         end
        %
        % This are for the path constraints
        %
        kk = 1;
        for ii = (nstates + 1):(nstates+npaths)
            daerows = nnodes*(ii-1)+1:nnodes*(ii);
            rows = (nnodes-1)*(ii-1)+kk:(nnodes-1)*(ii)+kk;
            % dc/dx
            for jj = 1:nstates
                cols    = nnodes*(jj-1)+1:nnodes*(jj);
                Jcon(rows,cols) = diag(Ddae_out(daerows,jj));
             end
            % dc/du
            for jj = nstates+1:nstates+ncontrols
                cols = (nnodes)*(nstates) + (nnodes*(jj-1-nstates)+1:nnodes*(jj-nstates));
                Jcon(rows,cols) = diag(Ddae_out(daerows,jj));
            end
            % dc/dparam
            for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
                cols = nnodes*(nstates+ncontrols) + jj -(nstates+ncontrols);
                Jcon(rows,cols) = Ddae_out(daerows,jj);
            end
            tt = 1:nnodes;
            jj = nstates + ncontrols + nparameters;
            % dc/dto
            cols = nnodes*(nstates+ncontrols) + nparameters+1;
            Jcon(rows,cols) = Ddae_out(daerows,jj+1).*(1-1/nnodes*tt');
            % dc/dtf
            cols = nnodes*(nstates+ncontrols) + nparameters+2;
            Jcon(rows,cols) = Ddae_out(daerows,jj+1).*(1+1/nnodes*tt'); 
            kk = kk+1;
        end
        %
        % This is for the event constraints
        %
        for ii = 1:nevents
            rows = (nnodes-1)*(npaths+nstates)+ ii + 1;
            % dEvent/dx
            for jj = 1:nstates
                col0 = (nnodes)*(jj-1)+1;
                colF = (nnodes)*(jj);
                Jcon(rows,col0) = Devents(ii,jj);
                Jcon(rows,colF) = Devents(ii,jj + nstates+1);
            end
            %dEvent/dparam
            for jj = (nstates+ncontrols) + 1 : (nstates+ncontrols) + nparameters
                cols = nnodes*(nstates+ncontrols) + jj -(nstates+ncontrols);
                Jcon(rows,cols) = Devents(1,2*(nstates+1)+jj-(nstates+ncontrols));
            end
            % dEvent/dt0
            cols = nnodes*(nstates+ncontrols)+nparameters+1;
            Jcon(rows,cols) = Devents(1,nstates+1);
            % dEvent/dtf
            cols = nnodes*(nstates+ncontrols)+nparameters+2;
            Jcon(rows,cols) = Devents(1,2*nstates+2); 
        end
        % Map constraint derivatives to sparsity pattern
            J(mysetup.constraint_indices{i},mysetup.variable_indices{i}) = Jcon;
end

end
%% Compute nonlinear and linear constraints

Constraints = vertcat(Cons{:,1});

linkages = mysetup.linkages;
if ~isempty(mysetup.linkages),
    link_out = cell(mysetup.numlinkpairs,1);
    link_row = length(Constraints)+1;
    
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
        sollink.left.time  = solTotal(left_phase).terminal.time;
        sollink.right.state = x0_right;
        sollink.right.parameter = p_right;
        sollink.right.time  = solTotal(right_phase).initial.time;
        sollink.right.phase = right_phase;
        
        if ~isequal(lower(mysetup.derivatives),'analytic')
            [link_out{ipair,1}] = feval(mysetup.funcs.link,sollink,mysetup);
        else
            [link_out{ipair,1} Dlink_out] = feval(mysetup.funcs.link,sollink,mysetup);
            %----------------------------%
            % Get Connection derivatives %
            %----------------------------%
            link_row = link_row(end)+ (1:nlinks);
            %[rows,cols] = find(mysetup.sparsity_all(link_row,:));
            %JJ = J(link_row',1:numel(Dlink_out(1,:)))
            J(link_row',1:numel(Dlink_out(1,:))) = Dlink_out(:,:);
        end
      
    end
    Clink = vertcat(link_out{:,1});
    Constraints = [Constraints; Clink];
end

C = [Constraints;mysetup.initlincons] + mysetup.Alinear_augmented*x;





