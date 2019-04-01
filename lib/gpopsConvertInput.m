function setup = gpopsConvertInput(setup);
%------------------------------------------------------------------%
% Converts input format from Cell array to Structures              %
%   for backward compatablitity with GPOPS versions 1.x            %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
global mysetup

convertInput = 0;
%------------------------------------------------------------------%
% Convert limits from Cell array to Structures                     %
%------------------------------------------------------------------%
if isfield(setup,'limits') && iscell(setup.limits)
    convertInput = 1;
    limitsTemp = setup.limits;
    setup = rmfield(setup,'limits');
    nodesTemp = setup.nodes;
    setup = rmfield(setup,'nodes');
    for iphase = 1:size(limitsTemp{1},1)
        setup.limits(iphase).nodes = nodesTemp(iphase);
        setup.limits(iphase).time.min = limitsTemp{1}{iphase,1};
        setup.limits(iphase).time.max = limitsTemp{2}{iphase,1};
        if size(limitsTemp{1},2) > 1 && size(limitsTemp{2},2) > 1
            setup.limits(iphase).state.min = limitsTemp{1}{iphase,2};
            setup.limits(iphase).state.max = limitsTemp{2}{iphase,2};
        end
        if size(limitsTemp{1},2) > 2 && size(limitsTemp{2},2) > 2
            setup.limits(iphase).control.min = limitsTemp{1}{iphase,3};
            setup.limits(iphase).control.max = limitsTemp{2}{iphase,3};
        else
            setup.limits(iphase).control.min = [];
            setup.limits(iphase).control.max = [];
        end
        if size(limitsTemp{1},2) > 3 && size(limitsTemp{2},2) > 3
            setup.limits(iphase).parameter.min = limitsTemp{1}{iphase,4};
            setup.limits(iphase).parameter.max = limitsTemp{2}{iphase,4};
        else
            setup.limits(iphase).parameter.min = [];
            setup.limits(iphase).parameter.max = [];
        end
        if size(limitsTemp{1},2) > 4 && size(limitsTemp{2},2) > 4
            setup.limits(iphase).path.min = limitsTemp{1}{iphase,5};
            setup.limits(iphase).path.max = limitsTemp{2}{iphase,5};
        else
            setup.limits(iphase).path.min = [];
            setup.limits(iphase).path.max = [];
        end
        if size(limitsTemp{1},2) > 5 && size(limitsTemp{2},2) > 5
            setup.limits(iphase).event.min = limitsTemp{1}{iphase,6};
            setup.limits(iphase).event.max = limitsTemp{2}{iphase,6};
        else
            setup.limits(iphase).event.min = [];
            setup.limits(iphase).event.max = [];
        end
        if size(limitsTemp{1},2) > 6 && size(limitsTemp{2},2) > 6
            setup.limits(iphase).duration.min = limitsTemp{1}{iphase,7};
            setup.limits(iphase).duration.max = limitsTemp{2}{iphase,7};
        else
            setup.limits(iphase).duration.min = [];
            setup.limits(iphase).duration.max = [];
        end
    end
    setup.limisTemp = limitsTemp;
    setup.nodesTemp = nodesTemp;
end
%------------------------------------------------------------------%
% Convert guess from Cell array to Structures                      %
%------------------------------------------------------------------%
if isfield(setup,'solinit') && iscell(setup.solinit)
    convertInput = 1;
    solinitTemp = setup.solinit;
    setup = rmfield(setup,'solinit');
    for iphase = 1:size(solinitTemp,1)
        setup.guess(iphase).time = solinitTemp{iphase,1};
        setup.guess(iphase).state = solinitTemp{iphase,2};
        setup.guess(iphase).control = solinitTemp{iphase,3};
        if size(solinitTemp,2) > 3 
            setup.guess(iphase).parameter = solinitTemp{iphase,4};
        else
            setup.guess(iphase).parameter = [];
        end
    end
    setup.solinitTemp = solinitTemp;
end
%------------------------------------------------------------------%
% Convert linkage from Cell array to Structures                    %
%------------------------------------------------------------------%
if isfield(setup,'connections') && iscell(setup.connections)
    convertInput = 1;
    connectionsTemp = setup.connections;
    setup = rmfield(setup,'connections');
    for ipair = 1:size(connectionsTemp,1)
        setup.linkages(ipair).left.phase = connectionsTemp{ipair,3}(1);
        setup.linkages(ipair).right.phase = connectionsTemp{ipair,3}(2);
        setup.linkages(ipair).min = connectionsTemp{ipair,1};
        setup.linkages(ipair).max = connectionsTemp{ipair,2};
    end
    setup.connectionsTemp = connectionsTemp;
elseif isfield(setup,'connections') && isempty(setup.connections)
    convertInput = 1;
    setup.linkages = setup.connections;
    setup.connectionsTemp = setup.connections;
    setup = rmfield(setup,'connections');
end
%------------------------------------------------------------------%
% Convert user functions from Cell array to Structures             %
%------------------------------------------------------------------%
if isfield(setup,'funcs') && iscell(setup.funcs)
    convertInput = 1;
    funcsTemp = setup.funcs;
    setup = rmfield(setup,'funcs');
    if length(funcsTemp) >= 1 && ~isempty(funcsTemp{1})
        setup.funcs.cost = 'gpopsConvertCost';
    else
        setup.funcs.cost = [];
    end
    if length(funcsTemp) >= 2 && ~isempty(funcsTemp{2})
        setup.funcs.dae = 'gpopsConvertDae';
    else
        setup.funcs.dae = [];
    end
    if length(funcsTemp) >= 3 && ~isempty(funcsTemp{3})
        setup.funcs.event = 'gpopsConvertEvent';
    else
        setup.funcs.event = [];
    end
    if length(funcsTemp) >= 4 && ~isempty(funcsTemp{4})
        setup.funcs.link = 'gpopsConvertLinkage';
    else
        setup.funcs.link = [];
    end
    setup.funcsTemp = funcsTemp;
    % set functions names to global variable so GPOPS can run error

    mysetup.funcsTemp = funcsTemp;
end
%------------------------------------------------------------------%
% Set flag for output                                              %
%------------------------------------------------------------------%
if convertInput == 1
    fprintf(['WARNING! Cell input format is no longer supported\n\t',...
        'error checking of user input may not function correctly\n\t',...
        'Convert interface to structure format (see manual for usage)\n\n']);
    setup.convertOutput = 1;
end
