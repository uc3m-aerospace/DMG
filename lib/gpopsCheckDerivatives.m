function gpopsCheckDerivatives(setup);
%------------------------------------------------------------------%
% Check the values of user defined analytic derivatives            %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin % 
%------------------------------------------------------------------%

fprintf('\nChecking user defined analytic derivatives against finite difference\n');
fprintf('------------------------------------------------------------------------\n');
guess = setup.guess;
numphases = setup.numphases;

if isfield(setup,'checkDerivativesEpsilon')
    epsilon = setup.checkDerivativesEpsilon;
else
    epsilon = 1E-6;
end
maxError = 0;
sizes = setup.sizes;
for iphase=1:numphases;
    % ------------------------------------------
    % Get the guess in each phase of the problem
    % ------------------------------------------
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
    %----------------------------------------------------------------------
    % check cost deriv 
    %----------------------------------------------------------------------
    clear sol
    sol.initial.time = tinit(1);
    sol.initial.state = xinit(1,:).';
    sol.terminal.time = tinit(end);
    sol.terminal.state = xinit(end,:).';
    sol.time = tinit;
    sol.state = xinit;
    sol.control = uinit;
    sol.parameter = pinit;
    sol.phase = iphase;
    % try user cost function
    [Mayer,Lagrange,DMayer,DLagrange] = feval(setup.funcs.cost,sol);
    [row, col] = size(DMayer);
    if row ~= 1 || col ~= 2*nstates+nparameters+2
        error(['Cost function "%s" returned invalid size of row vector for ',...
            'derivative of Mayer Cost in phase %i'],setup.funcs.cost,iphase)
    end
    [row, col] = size(DLagrange);
    if row ~= length(tinit)
        error(['Cost function "%s" returned invalid number of rows for ',...
            'derivative of Lagrange Cost in phase %i'],setup.funcs{1},iphase)
    end
    if col ~= nstates+ncontrols+nparameters+1
        error(['Cost function "%s" returned invalid number of columns for ',...
            'derivative of Lagrange Cost in phase %i'],setup.funcs{1},iphase)
    end
    %----------------------------------------------------------------------
    % estimate cost deriv using finite difference
    %----------------------------------------------------------------------
    DEMayer = zeros(1,2*nstates+nparameters+2);
    for ii = 1:nstates
        % check x0
        sol_p = sol;
        sol_p.initial.state(ii) = sol_p.initial.state(ii) + epsilon;
        [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
        DEMayer(1,ii) = (Mayer_p - Mayer)/epsilon;  
        % check xf
        sol_p = sol;
        sol_p.terminal.state(ii) = sol_p.terminal.state(ii) + epsilon;
        [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
        DEMayer(1,ii+nstates+1) = (Mayer_p - Mayer)/epsilon;  
    end
    % check t0
    sol_p = sol;
    sol_p.initial.time = sol_p.initial.time + epsilon;
    % sol_p{1,1} = sol_p{1,1} + epsilon;
    [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
    DEMayer(1,nstates+1) = (Mayer_p - Mayer)/epsilon;   
    % check tf
    sol_p = sol;
    sol_p.terminal.time = sol_p.terminal.time + epsilon;
    % sol_p{1,3} = sol_p{1,3} + epsilon;
    [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
    DEMayer(1,2*nstates+2) = (Mayer_p - Mayer)/epsilon; 
    for ii = 1:nparameters
        % check p
        sol_p = sol;
        sol_p.parameter(ii) = sol_p.parameter(ii) + epsilon;
        % sol_p{2,4}(ii) = sol_p{2,4}(ii) + epsilon;
        [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
        DEMayer(1,2*nstates+2+ii) = (Mayer_p - Mayer)/epsilon; 
    end
    fprintf('Phase %i, Mayer Cost:\n',iphase)
    for ii = 1:nstates
        fprintf('\t\tdC/dx0(%i):  \tUser = %12.5G,\tmax error = %12.5G\n',...
            ii,DMayer(ii),DMayer(ii)-DEMayer(ii));
    end
    fprintf('\t\tdC/dt0:     \tUser = %12.5G,\tmax error = %12.5G\n',...
        DMayer(nstates+1),DMayer(nstates+1)-DEMayer(nstates+1));
    for ii = 1:nstates
        fprintf('\t\tdC/dxf(%i):   \tUser = %12.5G,\tmax error = %12.5G\n',...
            ii,DMayer(1+nstates+ii),DMayer(1+nstates+ii)-DEMayer(1+nstates+ii));
    end
    fprintf('\t\tdC/dtf:     \tUser = %12.5G,\tmax error = %12.5G\n',...
        DMayer(nstates + 2),DMayer(nstates + 2)-DEMayer(nstates + 2));
    if nparameters > 0
        for ii = 1:nparameters
            fprintf('\t\tdC/dp(%i):    \tUser = %12.5G,\tmax error = %12.5G\n',...
                ii,DMayer(2+2*nstates+ii),DMayer(2+2*nstates+ii)-DEMayer(2+2*nstates+ii));
        end
    end
    maxError = max(maxError,max(abs(DMayer - DEMayer)));
    % check Lagrange Cost
    fprintf('Phase %i, Lagrange Cost:\n',iphase)
    N = length(tinit);
    DELagrange = zeros(N,nstates+ncontrols+nparameters+1);
    for ii = 1:nstates
        for jj = 1:N
            sol_p = sol;
            sol_p.state(jj,ii) = sol_p.state(jj,ii) + epsilon;
            [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
            DELagrange(jj,ii) = (Lagrange_p(jj) - Lagrange(jj))/epsilon; 
        end
        Lerror = DELagrange(:,ii) - DLagrange(:,ii); 
        I = find(isnan(Lerror),1);
        if isempty(I);
            I = find(abs(Lerror) == max(abs(Lerror)),1);
        end
        fprintf('\t\tdC/dx(%i):   \tUser = %12.5G,\tmax error = %12.5G\n',...
            ii,DLagrange(I,ii),Lerror(I));
        maxError = max(maxError,max(abs(Lerror)));
    end
    for ii = 1:ncontrols
        for jj = 1:N
            sol_p = sol;
            sol_p.control(jj,ii) = sol_p.control(jj,ii) + epsilon;
            [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
            DELagrange(jj,nstates+ii) = (Lagrange_p(jj) - Lagrange(jj))/epsilon; 
        end
        Lerror = DELagrange(:,nstates+ii) - DLagrange(:,nstates+ii);
        I = find(isnan(Lerror),1);
        if isempty(I);
            I = find(abs(Lerror) == max(abs(Lerror)),1);
        end
        fprintf('\t\tdC/du(%i):   \tUser = %12.5G,\tmax error = %12.5G\n',...
            ii,DLagrange(I,nstates+ii),Lerror(I));
        maxError = max(maxError,max(abs(Lerror)));
    end
    for ii = 1:nparameters
        sol_p = sol;
        sol_p.parameter(ii) = sol_p.parameter(ii) + epsilon;
        % sol_p{2,4}(ii) = sol_p{2,4}(ii) + epsilon;
        [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
        DELagrange(:,nstates+ncontrols+ii) = (Lagrange_p - Lagrange)/epsilon; 
        Lerror = DELagrange(:,nstates+ncontrols+ii) - DLagrange(:,nstates+ncontrols+ii);
        I = find(isnan(Lerror),1);
        if isempty(I);
            I = find(abs(Lerror) == max(abs(Lerror)),1);
        end
        fprintf('\t\tdC/dp(%i):   \tUser = %12.5G,\tmax error = %12.5G\n',...
            ii,DLagrange(I,nstates+ncontrols+ii),Lerror(I));
        maxError = max(maxError,max(abs(Lerror)));
    end
    for jj = 1:N
        sol_p = sol;
        sol_p.time(jj) = sol_p.time(jj) + epsilon;
        % sol_p{2,1}(jj) = sol_p{2,1}(jj) + epsilon;
        [Mayer_p, Lagrange_p] = feval(setup.funcs.cost,sol_p);
        DELagrange(jj,nstates+ncontrols+nparameters+ii) = (Lagrange_p(jj) - Lagrange(jj))/epsilon; 
    end
    Lerror = DELagrange(:,nstates+ncontrols+nparameters+1) - DLagrange(:,nstates+ncontrols+nparameters+1);
    I = find(isnan(Lerror),1);
    if isempty(I);
        I = find(abs(Lerror) == max(abs(Lerror)),1);
    end
    fprintf('\t\tdC/dt:      \tUser = %12.5G,\tmax error = %12.5G\n',...
        DLagrange(I,nstates+ncontrols+nparameters+1),Lerror(I));
    maxError = max(maxError,max(abs(Lerror)));
    %----------------------------------------------------------------------
    % check dae deriv 
    %----------------------------------------------------------------------
    clear sol
    sol.time = tinit;
    sol.state = xinit;
    sol.control = uinit;
    sol.parameter = pinit;
    sol.phase = iphase;
    % try user dae function
    [dae Ddae] = feval(setup.funcs.dae,sol);
    [row, col] = size(Ddae);
    if row ~= length(tinit)*(nstates+npaths)
        error(['Dae function "%s" returned invalid number of rows for ',...
            'derivative of Dae in phase %i'],setup.funcs.dae,iphase)
    end
    if col ~= nstates+ncontrols+nparameters+1
        error(['Dae function "%s" returned invalid number of columns ',...
            'for derivative of Dae in phase %i'],setup.funcs.dae,iphase)
    end
    %----------------------------------------------------------------------
    % estimate dae deriv using finite difference
    %----------------------------------------------------------------------
    DEdae = zeros(N*(nstates+npaths),nstates+ncontrols+nparameters+1);
    for ii = 1:nstates
        for jj = 1:N
            sol_p = sol;
            sol_p.state(jj,ii) = sol_p.state(jj,ii) + epsilon;
            [dae_p] = feval(setup.funcs.dae,sol_p);
            DEtot = (dae_p(jj,:) - dae(jj,:))/epsilon;
            for kk = 1:nstates+npaths
                row = (kk-1)*N+jj;
                DEdae(row,ii) = DEtot(kk); 
            end
        end
    end
    for ii = 1:ncontrols
        for jj = 1:N
            sol_p = sol;
            sol_p.control(jj,ii) = sol_p.control(jj,ii) + epsilon;
            % sol_p{3}(jj,ii) = sol_p{3}(jj,ii) + epsilon;
            [dae_p] = feval(setup.funcs.dae,sol_p);
            DEtot = (dae_p(jj,:) - dae(jj,:))/epsilon;
            for kk = 1:nstates+npaths
                row = (kk-1)*N+jj;
                DEdae(row,nstates+ii) = DEtot(kk); 
            end
        end
    end
    for ii = 1:nparameters
        sol_p = sol;
        sol_p.parameter(ii) = sol_p.parameter(ii) + epsilon;
        % sol_p{4}(ii) = sol_p{4}(ii) + epsilon;
        [dae_p] = feval(setup.funcs.dae,sol_p);
        DEtot = (dae_p(:,:) - dae(:,:))/epsilon;
        DEdae(:,nstate+nparamters+ii) = DEtot(:);
    end
    for jj = 1:N
        sol_p = sol;
        sol_p.time(ii) = sol_p.time(ii) + epsilon;
        % sol_p{1}(ii) = sol_p{1}(ii) + epsilon;
        [dae_p] = feval(setup.funcs.dae,sol_p);
        DEtot = (dae_p(jj,:) - dae(jj,:))/epsilon;
        for kk = 1:nstates+npaths
            row = (kk-1)*N+1+jj;
            DEdae(row,ii) = DEtot(kk); 
         end
    end
    fprintf('Phase %i, Dae Dynamics:\n',iphase)
    for ii = 1:nstates
        row = (ii-1)*N+1:ii*N;
        for jj = 1:nstates
            Derror = DEdae(row,jj) - Ddae(row,jj);
            I = find(isnan(Derror),1);
            if isempty(I);
                I = find(abs(Derror) == max(abs(Derror)),1);
            end
            fprintf('\t\tdf(%i)/dx(%i):\tUser = %12.5G,\tmax error = %12.5G\n',ii,jj,Ddae(I+row(1)-1,jj),Derror(I));
            maxError = max(maxError,max(abs(Derror)));
        end
        for jj = 1:ncontrols
            Derror = DEdae(row,jj+nstates) - Ddae(row,jj+nstates);
            I = find(isnan(Derror),1);
            if isempty(I);
                I = find(abs(Derror) == max(abs(Derror)),1);
            end
            fprintf('\t\tdf(%i)/du(%i):\tUser = %12.5G,\tmax error = %12.5G\n',...
                ii,jj,Ddae(I+row(1)-1,jj+nstates),Derror(I));
            maxError = max(maxError,max(abs(Derror)));
        end
        for jj = 1:nparameters
            Derror = DEdae(row,jj+nstates+ncontrols) - Ddae(row,jj+nstates+ncontrols);
            I = find(isnan(Derror),1);
            if isempty(I);
                I = find(abs(Derror) == max(abs(Derror)),1);
            end
            fprintf('\t\tdf(%i)/dp(%i):\tUser = %12.5G,\tmax error = %12.5G\n',...
                ii,jj,Ddae(I+row(1)-1,jj+nstates+ncontrols),Derror(I));
            maxError = max(maxError,max(abs(Derror)));
        end
        Derror = DEdae(row,nparameters+nstates+ncontrols+1) - Ddae(row,nparameters+nstates+ncontrols+1);
        I = find(isnan(Derror),1);
        if isempty(I);
            I = find(abs(Derror) == max(abs(Derror)),1);
        end
        fprintf('\t\tdf(%i)/dt:    \tUser = %12.5G,\tmax error = %12.5G\n',...
            ii,Ddae(I+row(1)-1,nparameters+nstates+ncontrols+1),Derror(I));
        maxError = max(maxError,max(abs(Derror)));
    end
    if npaths > 0; fprintf('Phase %i, Dae Path:\n',iphase); end
    for ii = 1:npaths
        row = (nstates+ii-1)*N+1:(nstates+ii)*N;
        for jj = 1:nstates
            Derror = DEdae(row,jj) - Ddae(row,jj);
            I = find(isnan(Derror),1);
            if isempty(I);
                I = find(abs(Derror) == max(abs(Derror)),1);
            end
            fprintf('\t\tdp(%i)/dx(%i):\tUser = %12.5G,\tmax error = %12.5G\n',...
                ii,jj,Ddae(I+row(1)-1,jj),Derror(I));
            maxError = max(maxError,max(abs(Derror)));
        end
        for jj = 1:ncontrols
            Derror = DEdae(row,jj+nstates) - Ddae(row,jj+nstates);
            I = find(isnan(Derror),1);
            if isempty(I);
                I = find(abs(Derror) == max(abs(Derror)),1);
            end
            fprintf('\t\tdp(%i)/du(%i):\tUser = %12.5G,\tmax error = %12.5G\n',...
                ii,jj,Ddae(I+row(1)-1,jj+nstates),Derror(I));
            maxError = max(maxError,max(abs(Derror)));
        end
        for jj = 1:nparameters
            Derror = DEdae(row,jj+nstates+ncontrols) - Ddae(row,jj+nstates+ncontrols);
            I = find(isnan(Derror),1);
            if isempty(I);
                I = find(abs(Derror) == max(abs(Derror)),1);
            end
            fprintf('\t\tdp(%i)/dp(%i):\tUser = %12.5G,\tmax error = %12.5G\n',...
                ii,jj,Ddae(I+row(1)-1,jj+nstates+ncontrols),Derror(I));
            maxError = max(maxError,max(abs(Derror)));
        end
        Derror = DEdae(row,nparameters+nstates+ncontrols+1) - Ddae(row,nparameters+nstates+ncontrols+1);
        I = find(isnan(Derror),1);
        if isempty(I);
            I = find(abs(Derror) == max(abs(Derror)),1);
        end
        fprintf('\t\tdp(%i)/dt:    \tUser = %12.5G,\tmax error = %12.5G\n',ii,...
            Ddae(I+row(1)-1,nparameters+nstates+ncontrols+1),Derror(I));
        maxError = max(maxError,max(abs(Derror)));
    end
    % ------------------%
    % Check Event deriv %
    % ------------------%
    if nevents > 0
        clear sol
        sol.initial.time = tinit(1);
        sol.initial.state = xinit(1,:).';
        sol.terminal.time = tinit(end);
        sol.terminal.state = xinit(end,:).';
        sol.parameter = pinit;
        sol.phase = iphase;
        % try user Event function
        [event Devent] = feval(setup.funcs.event,sol);
        [row, col] = size(Devent);
        if row ~= nevents
            error(['Event function "%s" returned invalid number of rows ',...
                'for derivative of Events in phase %i'],setup.funcs{3},iphase)
        end
        if col ~= 2*nstates+nparameters+2
            error(['Event function "%s" returned invalid number of columns ',...
                'for derivative of Events in phase %i'],setup.funcs{3},iphase)
        end
        %----------------------------------------------------------------------
        % estimate event deriv using finite difference
        %----------------------------------------------------------------------
        DEevent = zeros(nevents,2*nstates+nparameters+2);
        for ii = 1:nstates
            % check x0
            sol_p = sol;
            sol_p.initial.state(ii) = sol_p.initial.state(ii) + epsilon;
            [event_p Devent_p] = feval(setup.funcs.event,sol_p);
            DEevent(:,ii) = (event_p - event)/epsilon;  
            % check xf
            sol_p = sol;
            sol_p.terminal.state(ii) = sol_p.terminal.state(ii)+epsilon;
            [event_p Devent_p] = feval(setup.funcs.event,sol_p);
            DEevent(:,ii+nstates+1) = (event_p - event)/epsilon;   
        end
        % check t0
        sol_p = sol;
        sol_p.initial.time = sol_p.initial.time + epsilon;
        [event_p Devent_p] = feval(setup.funcs.event,sol_p);
        DEevent(:,nstates+1) = (event_p - event)/epsilon; 
        % check tf
        sol_p = sol;
        sol_p.terminal.time = sol_p.terminal.time + epsilon;
        [event_p Devent_p] = feval(setup.funcs.event,sol_p);
        DEevent(:,2*nstates+2) = (event_p - event)/epsilon;
        for ii = 1:nparameters
            % check p
            sol_p = sol;
            sol_p.parameter(ii) = sol_p.parameter(ii) + epsilon;
            [event_p Devent_p] = feval(setup.funcs.event,sol_p);
            DEevent(:,2*nstates+2+ii) = (event_p - event)/epsilon;
        end
        fprintf('Phase %i, Events:\n',iphase)
        for jj = 1:nevents
            for ii = 1:nstates
                fprintf('\t\tdE(%i)/dx0(%i):\tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                    Devent(jj,ii),Devent(jj,ii)-DEevent(jj,ii));
            end
            fprintf('\t\tdE(%i)/dt0:   \tUser = %12.5G,\tmax error = %12.5G\n',jj,...
                Devent(jj,nstates+1),Devent(jj,nstates+1)-DEevent(jj,nstates+1));
            for ii = 1:nstates
                fprintf('\t\tdE(%i)/dxf(%i): \tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                    Devent(jj,1+nstates+ii),Devent(jj,1+nstates+ii)-DEevent(jj,1+nstates+ii));
            end
            fprintf('\t\tdE(%i)/dtf:   \tUser = %12.5G,\tmax error = %12.5G\n',jj,...
                Devent(jj,2*nstates + 1),Devent(jj,2*nstates + 1)-DEevent(jj,2*nstates + 1));
            if nparameters > 0
                for ii = 1:nparameters
                    fprintf('\t\tdE(%i)/dp(%i):  \tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                        Devent(jj,2+2*nstates+ii),Devent(jj,2+2*nstates+ii)-DEevent(jj,2+2*nstates+ii));
                end
            end
            maxError = max(maxError,max(abs(Devent(jj,:) - DEevent(jj,:))));
        end
    end
end
%---------------------%
% Check Connect deriv %
%---------------------%
numlinkpairs = setup.numlinkpairs;
linkages = setup.linkages;
for ipair = 1:numlinkpairs
    fprintf('Connection %i:\n',ipair)
    phaseL = linkages(ipair).left.phase;
    phaseR = linkages(ipair).right.phase;
    nlink = length(linkages(ipair).min);
    clear sol
    sol.left.phase = phaseL;
    sol.left.state = guess(phaseL).state(end,:).';
    sol.left.parameter = guess(phaseL).parameter;
    sol.right.phase = phaseR;
    sol.right.state = guess(phaseR).state(end,:).';
    sol.right.parameter = guess(phaseR).parameter;
    % try user Linkage function
    [connect Dconnect] = feval(setup.funcs.link,sol);
    nstatesL = length(sol.left.state);
    nstatesR = length(sol.right.state);
    nparametersL = length(sol.left.parameter);
    nparametersR = length(sol.right.parameter);
    [row, col] = size(Dconnect);
    if row ~= nlink
        error(['Connection function "%s" returned invalid number of rows ',...
            'for derivative of Links for connection %i'],...
            setup.funcs.link,ipair)
    end
    if col ~= nstatesL+nstatesR+nparametersL+nparametersR
        error(['Connection function "%s" returned invalid number of columns ',...
            ' for derivative of Link for connection %i'],...
            setup.funcs.link,ipair)
    end
    DEconnect = zeros(nlink,nstatesL+nstatesR+nparametersL+nparametersR);
    for ii = 1:nstatesL
        % check xf left
        sol_p = sol;
        sol_p.left.state = sol_p.left.state + epsilon;
        [connect_p Dconnect_p] = feval(setup.funcs.link,sol_p);
        DEconnect(:,ii) = (connect_p - connect)/epsilon;        
    end
    for ii = 1:nparametersL
        % check p left
        sol_p = sol;
        sol_p.left.parameter = sol_p.left.parameter + epsilon;
        [connect_p Dconnect_p] = feval(setup.funcs.link,sol_p);
        DEconnect(:,nstatesL+ii) = (connect_p - connect)/epsilon;
    end
    for ii = 1:nstatesR
        % check x0 right
        sol_p = sol;
        sol_p.right.state = sol_p.right.state + epsilon;
        [connect_p Dconnect_p] = feval(setup.funcs.link,sol_p);
        DEconnect(:,ii+nstatesL+nparametersL) = (connect_p - connect)/epsilon;
    end
    for ii = 1:nparametersR
        % check p right
        sol_p = sol;
        sol_p.right.parameter = sol_p.right.parameter + epsilon;
        [connect_p Dconnect_p] = feval(setup.funcs.link,sol_p);
        DEconnect(:,nstatesL+nparametersL+nstatesR+ii) = (connect_p - connect)/epsilon;
    end
    for jj = 1:nlink
        for ii = 1:nstatesL
            fprintf('\t\tdL(%i)/dxfL(%i):\tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                Dconnect(jj,ii),Dconnect(jj,ii)-DEconnect(jj,ii));
        end
        for ii = 1:nparametersL
            fprintf('\t\tdL(%i)/dpL(%i):\tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                Dconnect(jj,ii+nstatesL),Dconnect(jj,ii+nstatesL)-DEconnect(jj,ii+nstatesL));
        end
        for ii = 1:nstatesR
            fprintf('\t\tdL(%i)/dx0R(%i):\tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                Dconnect(jj,ii+nstatesL+nparametersL),...
                Dconnect(jj,ii+nstatesL+nparametersL)-DEconnect(jj,ii+nstatesL+nparametersL));
        end
        for ii = 1:nparametersR
            fprintf('\t\tdL(%i)/dpR(%i):\tUser = %12.5G,\tmax error = %12.5G\n',jj,ii,...
                Dconnect(jj,ii+nstatesL+nparametersL+nstatesR),...
                Dconnect(jj,ii+nstatesL+nparametersL+nstatesR)...
                   -DEconnect(jj,ii+nstatesL+nparametersL+nstatesR));
        end  
        maxError = max(maxError,max(abs(Dconnect(jj,:) - DEconnect(jj,:))));
    end
    
end
fprintf('\n\nMax Derivative Error = %12.5G\n',maxError)
fprintf('---------------------------------------------\n\n')
