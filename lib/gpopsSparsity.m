function setup = gpopsSparsity(setup);
%------------------------------------------------------------------%
% Generate the sparsity pattern for a multiple-phase optimal       %
% control setup discretized using the Gauss pseudospectral         %
% method.                                                          %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%


sizes = setup.sizes;
nodes = setup.nodes;
numphases = setup.numphases;
dependencies = setup.dependencies;
rowshift = 0;
colshift = 0;

for i=1:numphases;
    nstates = sizes(i,1);
    ncontrols = sizes(i,2);
    nparameters = sizes(i,3);
    npaths = sizes(i,4);
    nevents = sizes(i,5);
    phase_info = [nodes(i); nstates; ncontrols; nparameters; npaths; nevents];
    [Pattern(i).S,Pattern(i).Sjac,Pattern(i).Sconstant] = gpopsPhaseSparsity(i,setup,phase_info,dependencies{i});
    [rows,cols] = size(Pattern(i).S);
    row_indices = rowshift+1:rowshift+rows;
    col_indices = colshift+1:colshift+cols;
    S(row_indices,col_indices) = Pattern(i).S;
    Sjac(row_indices,col_indices) = Pattern(i).Sjac;
    Sconstant(row_indices,col_indices) = Pattern(i).Sconstant;
    rowshift = rowshift+rows;
    colshift = colshift+cols;
end;
% ----------------------------------------
% Sparsity Pattern for Linkage Conditions
% ----------------------------------------
linkages = setup.linkages;
numlinkpairs = setup.numlinkpairs;
numvars = setup.numvars;
variable_indices = setup.variable_indices;
Slink = [];
for ipair=1:numlinkpairs;
    left_phase  = linkages(ipair).left.phase;
    right_phase = linkages(ipair).right.phase;
    nodes_left = nodes(left_phase);
    disc_left  = nodes_left+2;
    nstates_left = sizes(left_phase,1);
    ncontrols_left = sizes(left_phase,2);
    nparameters_left = sizes(left_phase,3);
    nodes_right = nodes(right_phase);
    disc_right  = nodes_right+2;
    nstates_right = sizes(right_phase,1);
    ncontrols_right = sizes(right_phase,2);
    nparameters_right = sizes(right_phase,3);
    indices_left = disc_left:disc_left:nstates_left*disc_left;
    nparameters_left_start = (nodes_left+2)*nstates_left+nodes_left*ncontrols_left+3;
    nparameters_left_finish = (nodes_left+2)*nstates_left+nodes_left*ncontrols_left+2+nparameters_left;
    parameter_indices_left = nparameters_left_start:nparameters_left_finish;
    indices_left = [indices_left parameter_indices_left];
    indices_right = 1:disc_right:nstates_right*disc_right;
    nparameters_right_start = (nodes_right+2)*nstates_right+nodes_right*ncontrols+3;
    nparameters_right_finish = (nodes_right+2)*nstates_right+nodes_right*ncontrols_right+2+nparameters_right;
    parameter_indices_right = nparameters_right_start:nparameters_right_finish;
    indices_right = [indices_right parameter_indices_right];
    indices_left_use = variable_indices{left_phase}(indices_left);
    indices_right_use = variable_indices{right_phase}(indices_right);
    numlinks = length(linkages(ipair).min);
    Slinkcurr = zeros(numlinks,numvars);
    Slinkcurr(:,indices_left_use) = 1;
    Slinkcurr(:,indices_right_use) = 1;
    Slink = [Slink; Slinkcurr];
end;
S = [S; Slink];
S = sparse(S);
Sjac = [Sjac; Slink];
Sjac = sparse(Sjac);
Sconstant = [Sconstant; zeros(size(Slink))];
Sconstant = sparse(Sconstant);
setup.sparsity_nonlinear = S;
setup.sparsity_jac = Sjac;
setup.sparsity_constant = Sconstant;
setup.sparsity_all = [ones(1,size(S,2)); Sjac; zeros(size(setup.Alinear))];
