function setup = dmg_ScaleNlp_coll(setup)
%------------------------------------------------------------------%
% Determine the row and column scales for a non-sequential         %
% multiple-phase optimal control problem                           %
%------------------------------------------------------------------%
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
nodes = setup.nodes;
sizes = setup.sizes;
constraint_indices = setup.constraint_indices;
limits = setup.limits;
xmin = setup.varbounds_min;
xmax = setup.varbounds_max;

numvars = setup.numvars;
numnonlin = setup.numnonlin;
numphases = setup.numphases;

colscales     = ones(numvars,1);
rowscales     = ones(numnonlin,1);
dependencies  = cell(1,numphases);
%
%Oonly run if using autoscale
%
if isfield(setup,'autoscale') && isequal(setup.autoscale,'on')
    % ---------------------------
    % Get the column scales first
    % ---------------------------
    minmaxnotzero = find((xmin~=0) & (xmax~=0) & ~isinf(xmin) & ~isinf(xmax));
    colscales(minmaxnotzero)          = max(abs(xmin(minmaxnotzero)),abs(xmax(minmaxnotzero)));
    minzero                           = find((xmin==0) & (xmax~=0) & ~isinf(xmax));
    maxzero                           = find((xmin~=0) & ~isinf(xmin) & (xmax==0));
    colscales(minzero)            = abs(xmax(minzero));
    colscales(maxzero)            = abs(xmin(maxzero));
    mininfmaxnotzero                = find(isinf(xmin) & ~isinf(xmax) & (xmax~=0));
    colscales(mininfmaxnotzero) = abs(xmax(mininfmaxnotzero));
    maxinfminnotzero                = find(isinf(xmax) & ~isinf(xmin) & (xmin~=0));
    colscales(maxinfminnotzero) = abs(xmin(maxinfminnotzero));
    % --------------------------
    % Now compute the row scales
    % --------------------------
    BIG_NUM = 1E20;
    rand('state',0);
    ntrials = 93;
    extras.setup = setup;
    xupall = cell(1,numphases);
    phasecons = zeros(1,numphases);
    for iphase=1:numphases;
        tlowlimits = limits(iphase).time.min;
        tupplimits = limits(iphase).time.max;
        xlowlimits = limits(iphase).state.min;
        xupplimits = limits(iphase).state.max;
        ulowlimits = limits(iphase).control.min;
        uupplimits = limits(iphase).control.max;
        plowlimits = limits(iphase).parameter.min;
        pupplimits = limits(iphase).parameter.max;
        nstates = sizes(iphase,1);
        ncontrols = sizes(iphase,2);
        nparameters = sizes(iphase,3);
        npaths = sizes(iphase,4);
        nevents = sizes(iphase,5);
        dependencies{iphase} = zeros(nstates+npaths,nstates+ncontrols);
        colscales_curr = colscales(setup.variable_indices{1,iphase});
        state_scales = colscales_curr(1:(nodes(iphase))*nstates);
        state_scales_matrix = reshape(state_scales,nodes(iphase),nstates);
        state_scales_use = state_scales_matrix(2,:);
        state_scales_matrix = repmat(state_scales_use,nodes(iphase)-1,1);
        % --------------------------------------
        % Set bounds and remove infinite values
        % --------------------------------------
        tlow = tlowlimits(1);
        tlow(isinf(tlow)) = BIG_NUM * sign(tlow(isinf(tlow)));
        tupp = tupplimits(2);
        tupp(isinf(tupp)) = BIG_NUM * sign(tupp(isinf(tupp)));
        xlow = xlowlimits(:,2).';
        xlow(isinf(xlow)) = BIG_NUM * sign(xlow(isinf(xlow)));
        xupp = xupplimits(:,2).';
        xupp(isinf(xupp)) = BIG_NUM * sign(xupp(isinf(xupp)));
        ulow = ulowlimits.';
        ulow(isinf(ulow)) = BIG_NUM * sign(ulow(isinf(ulow)));
        uupp = uupplimits.';
        uupp(isinf(uupp)) = BIG_NUM * sign(uupp(isinf(uupp)));
        plow = plowlimits.';
        plow(isinf(plow)) = BIG_NUM * sign(plow(isinf(plow)));
        pupp = pupplimits.';
        pupp(isinf(pupp)) = BIG_NUM * sign(pupp(isinf(pupp)));
        % ------------------------------------------------
        % Generate random sample of variables using bounds 
        % ------------------------------------------------
        randtime = rand(ntrials,1);
        trand = randtime*tupp+(1-randtime)*tlow;
        if nstates>0
            randstate = rand(nstates,ntrials).';
            xrand = randstate.*repmat(xupp,ntrials,1)+(1-randstate).*repmat(xlow,ntrials,1);
        else
            xrand = [];
        end
        if ncontrols>0
            randcontrol = rand(ncontrols,ntrials).';
            urand = randcontrol.*repmat(uupp,ntrials,1)+(1-randcontrol).*repmat(ulow,ntrials,1);
        else
            urand = [];
        end
        if nparameters>0
            randparameters = rand(nparameters,ntrials).';
            prand = randparameters.*repmat(pupp,ntrials,1)+(1-randparameters).*repmat(plow,ntrials,1);
        else
            prand = [];
        end
        % ---------------------%
        % Initialize variables %
        % ---------------------%
        xuptot = zeros(nstates+ncontrols+nparameters,ntrials);
        dae_norm = zeros(nstates+npaths,ntrials);
        event_norm = zeros(nevents,ntrials);
        extras.phase = iphase;
        extras.nstates = nstates;
        extras.ncontrols = ncontrols;
        extras.nparameters = nparameters;
        for j=1:ntrials
            % ----------------------------------%
            % Evaluate the Jacobian of the DAEs %
            % ----------------------------------%
            t = trand(j);
            xup = zeros(nstates+ncontrols+nparameters,1);
            if nstates>0
                xup(1:nstates) = xrand(j,:).';
            end
            if ncontrols>0
                xup(nstates+1:nstates+ncontrols) = urand(j,:).';
            end
            if nparameters>0
                xup(nstates+ncontrols+1:nstates+ncontrols+nparameters) = prand(j,:).';
            end
            xuptot(:,j) = xup;
            % ---------------------------------------%
            % Evaluate Jacobian using random numbers %
            % ---------------------------------------%
            fty = feval('gpopsDaeWrapper',t,xup,extras);
            thresh = 1e-6*ones(size(xup));
            [daejac] = numjac('gpopsDaeWrapper',t,xup,fty,thresh,[],0,[],[],extras);
            daejac(isnan(daejac)) = 1;
            dae_norm(:,j) = sqrt(dot(daejac,daejac,2));
            % ------------------------------------%
            % Evaluate the Jacobian of the Events %
            % ------------------------------------%
            if nevents>0
                xupevent = zeros(2+2*nstates+nparameters,1);
                if nstates>0
                    initevent = [t; xrand(j,:).'];
                    termevent = [t; xrand(j,:).'];
                    xupevent(1:2+2*nstates) = [initevent; termevent];
                end
                if nparameters>0
                    xupevent(2+2*nstates+1:end) = prand(j,:).';
                end
                fty = feval('gpopsEventWrapper',t,xupevent,extras);
                thresh = 1e-6*ones(size(xupevent));
                [eventjac] = numjac('gpopsEventWrapper',t,xupevent,fty,thresh,[],0,[],[],extras);
                event_norm(:,j) = sqrt(dot(eventjac,eventjac,2));
            end
        end
        xupall{iphase} = xuptot;
        
        if (nstates+npaths)>0
            dae_norm_average = mean(dae_norm,2);
        end
        if nstates>0
            ode_norm_matrix = state_scales_matrix;
        else
            ode_norm_matrix = [];
        end
        if npaths>0
            path_norm_average = dae_norm_average(nstates+1:end);
            path_norm_average(logical(path_norm_average<eps)) = 1;
            path_norm_matrix = repmat(path_norm_average,1,nodes(iphase)).';
        else
            path_norm_matrix = [];
        end
        dae_norm_vector = [ode_norm_matrix(:); path_norm_matrix(:)];
        if nevents>0
            event_norm_vector = mean(event_norm,2);
            event_norm_vector(logical(event_norm_vector<eps)) = 1;
        else
            event_norm_vector = [];
        end
        rowscales(constraint_indices{1,iphase}) = [dae_norm_vector; event_norm_vector];
        phasecons(iphase) = (nodes(iphase)-1)*nstates+nodes(iphase)*npaths+nevents;
    end
    % -----------------------------
    % Determine the linkage scales
    % -----------------------------
    numlinkpairs = setup.numlinkpairs;
    linkages = setup.linkages;
    link_scales = [];
    last_phase_index = sum(phasecons);
    linkage_indices = (last_phase_index+1:numnonlin).';
    t = 0;
    for ipair=1:numlinkpairs
        left_phase               = linkages(ipair).left.phase;
        right_phase               = linkages(ipair).right.phase;
        nstates_left             = sizes(left_phase,1);
        nstates_right            = sizes(right_phase,1);
        ncontrols_left           = sizes(left_phase,2);
        ncontrols_right          = sizes(right_phase,2);
        nparameters_left         = sizes(left_phase,3);
        nparameters_right        = sizes(right_phase,3);
        nlinks                   = length(linkages(ipair).min);
        extras.left.phase        = left_phase;
        extras.left.nstates      = nstates_left;
        extras.left.nparameters  = nparameters_left;
        extras.right.phase       = right_phase;
        extras.right.nstates     = nstates_right;
        extras.right.nparameters = nparameters_right;
        link_norm                = zeros(nlinks,ntrials);
        for j=1:ntrials
            xupleft  = xupall{left_phase}(:,ipair);
            xupright = xupall{right_phase}(:,ipair);
            xleft = xupleft(1:nstates_left);
            pleft = xupleft(nstates_left+ncontrols_left+1:end);
            xright = xupright(1:nstates_right);
            pright = xupright(nstates_right+ncontrols_right+1:end);
            xplink = [xleft; pleft; xright; pright];
            fty = feval('gpopsLinkWrapper',t,xplink,extras);
            thresh = 1e-6*ones(size(xplink));
            [linkjac] = numjac('gpopsLinkWrapper',t,xplink,fty,thresh,[],0,[],[],extras);
            link_norm(:,j) = sqrt(dot(linkjac,linkjac,2));
        end
        link_norm_average = mean(link_norm,2);
        link_norm_average(logical(link_norm_average<eps)) = 1;
        link_scales = [link_scales; link_norm_average];
    end
    rowscales(linkage_indices) = link_scales;
end     %isfield(setup,'autoscale') && isequal(setup.autoscale,'on')
setup.column_scales = 1./colscales;
setup.row_scales = 1./rowscales;
% ---------------------------------
% Find worst-case sparsity pattern
% ---------------------------------
for iphase = 1:numphases
    % dependencies = (nstates + npaths) X (nstates + ncontrols)
    dependencies{iphase} = ones(sizes(iphase,1)+sizes(iphase,4),...
                                sizes(iphase,1)+sizes(iphase,2));
end
% ---------------------------------
% Check for user sparsity pattern
% ---------------------------------
if ~isfield(setup,'dependencies')
    % use calculated dependencies
    fprintf('\nUsing Default Sparsity\n')
    setup.dependencies = dependencies;
else
    % use user defined dependencies
    fprintf('\nUsing User Defined Sparsity\n')
    for iphase=1:numphases
        if isempty(setup.dependencies{iphase})
            setup.dependencies{iphase} = dependencies{iphase};
        else
            % check size
			% [drow,dcol] = size(dependencies{iphase});
            %drow = nstates + npaths
            drow = sizes(iphase,1) + sizes(iphase,4); 
            %dcol = nstates + ncontrols
            dcol = sizes(iphase,1) + sizes(iphase,2); 
            [drow_in,dcol_in] = size(setup.dependencies{iphase});
            if ~isequal(drow,drow_in) || ~isequal(dcol,dcol_in)
                error(['User defined sparsity in phase %i has incorrect size:',...
                       '\n\tUser: (%i X %i) Needs: (%i X %i)\n'],...
                       iphase,drow_in,dcol_in,drow,dcol)
            end
        end
    end
end
