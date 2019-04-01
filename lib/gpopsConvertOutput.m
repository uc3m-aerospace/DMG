function setup = gpopsConvertOutput(setup);
%------------------------------------------------------------------%
% Converts output format from Structures to Cell array             %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

if isfield(setup,'convertOutput') && setup.convertOutput == 1
    fprintf(['\nWARNING! Converting output from structure format to ',...
        'cell array for backwards compatability\n\t',...
        'Use structure format (see manual for usage)\n\n']);
    solutionTemp = setup.solution;
    setup = rmfield(setup,'solution');
    for iphase = 1:length(solutionTemp)
        setup.solution{iphase,1} = solutionTemp(iphase).time;
        setup.solution{iphase,2} = solutionTemp(iphase).state;
        setup.solution{iphase,3} = solutionTemp(iphase).control;
        setup.solution{iphase,4} = solutionTemp(iphase).parameter;
        setup.solution{iphase,5} = solutionTemp(iphase).costate;
        setup.solution{iphase,6} = solutionTemp(iphase).Hamiltonian;
        setup.solution{iphase,7} = solutionTemp(iphase).pathmult;
        setup.solution{iphase,8} = solutionTemp(iphase).Mayer_cost;
        setup.solution{iphase,9} = solutionTemp(iphase).Lagrange_cost;
    end
    setup.solution(length(solutionTemp)+1,:) = {'time','states',...
        'controls','parameters','costates','path_multipliers',...
        'Hamiltonian','Mayer_cost','Lagrange_cost'};
    %--------------------------------------------------------------%
    %    Set inputs for backwards compatability                    %
    %--------------------------------------------------------------%
    if isfield(setup,'limitsTemp')
        setup = rmfield(setup,'limits');
        setup.limits = setup.limitTemp;
        setup = rmfield(setup,'limitsTemp');
    end
    if isfield(setup,'solinitTemp')
        setup = rmfield(setup,'guess');
        setup.solinit = setup.solinitTemp;
        setup = rmfield(setup,'solinitTemp');
    end
    if isfield(setup,'connectionsTemp')
        setup = rmfield(setup,'linkages');
        setup.connections = setup.connectionsTemp;
        setup = rmfield(setup,'connectionsTemp');
    end
    if isfield(setup,'funcsTemp')
        setup = rmfield(setup,'funcs');
        setup.funcs = setup.funcsTemp;
        setup = rmfield(setup,'funcsTemp');
    end
end