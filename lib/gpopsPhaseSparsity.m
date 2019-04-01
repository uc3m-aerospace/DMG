function [S,Sjac,Sconstant] = gpopsPhaseSparsity(iphase,setup,phase_info,dependencies);
%------------------------------------------------------------------%
% Compute sparsity pattern for a single phase                      %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

nodes = phase_info(1);
nstates = phase_info(2);
ncontrols = phase_info(3);
nparameters = phase_info(4);
npaths = phase_info(5);
nevents = phase_info(6);
disc_pts = nodes+2;
ndiffeqs = nstates;
numvars = nstates*disc_pts+ncontrols*nodes+nparameters+2;
numcons = ndiffeqs*(nodes+1)+npaths*nodes+nevents;
S = sparse(numcons,numvars);
Sjac = S;
Sconstant = Sjac;
DQuadBlock = ones(nodes+1,nodes+2);
DdiagQuadBlock = zeros(nodes+1,nodes+2);
DdiagQuadBlock(1:nodes,2:nodes+1) = eye(nodes);
DiffMatOffDiag = zeros(nodes+1,nodes+2);
DiffMatOffDiag(1:nodes,1:nodes+1) = setup.ps(iphase).D-setup.ps(iphase).Ddiag;
DiffMatOffDiag(nodes+1,1:nodes+1) = -setup.ps(iphase).Weights*setup.ps(iphase).D;
DiffMatOffDiag(nodes+1,1) = DiffMatOffDiag(nodes+1,1) - 1;
DiffMatOffDiag(nodes+1,nodes+2) = 1;
OffDiag_State = zeros(nodes+1,nodes+2);
OffDiag_State(1:nodes,2:nodes+1) = eye(nodes);
OffDiag_State_Zero = zeros(nodes+1,nodes+2);
Diffeq_Block_Control = zeros(nodes+1,nodes);
Diffeq_Block_Control(1:nodes,1:nodes) = eye(nodes);
OffDiag_Block_Control_Zero = zeros(nodes+1,nodes);
Path_Block_State = zeros(nodes,nodes+2);
Path_Block_State(:,2:nodes+1) = eye(nodes);
Path_Block_State_Zero = zeros(nodes,nodes+2);
Path_Block_Control = eye(nodes);
Path_Block_Control_Zero = zeros(nodes,nodes);

for i=1:ndiffeqs;
    rowstart = (i-1)*(nodes+1)+1;
    rowfinish = i*(nodes+1);
    row_indices = rowstart:rowfinish;
    for j=1:nstates;
        colstart = (j-1)*disc_pts+1;
        colfinish = j*disc_pts;
        col_indices = colstart:colfinish;
        if isequal(i,j),
            S(row_indices,col_indices) = DQuadBlock;
            Sjac(row_indices,col_indices) = DdiagQuadBlock;
            Sconstant(row_indices,col_indices) = DiffMatOffDiag;
        else
            if isequal(dependencies(i,j),1),
                S(row_indices,col_indices) = OffDiag_State;
                Sjac(row_indices,col_indices) = OffDiag_State;
            else
                S(row_indices,col_indices) = OffDiag_State_Zero;
                Sjac(row_indices,col_indices) = OffDiag_State_Zero;
            end;
        end;
    end;
    colshift = nstates*disc_pts;
    for j=1:ncontrols;
        colstart = colshift+(j-1)*nodes+1;
        colfinish = colshift+j*nodes;
        col_indices = colstart:colfinish;
        if isequal(dependencies(i,j+nstates),1)
            S(row_indices,col_indices) = Diffeq_Block_Control;
            Sjac(row_indices,col_indices) = Diffeq_Block_Control;    
        else
            S(row_indices,col_indices) = OffDiag_Block_Control_Zero;
            Sjac(row_indices,col_indices) = OffDiag_Block_Control_Zero;
        end;
    end;
end;
rowshift = ndiffeqs*(nodes+1);
for i=1:npaths;
    rowstart = rowshift+(i-1)*nodes+1;
    rowfinish = rowshift+i*nodes;
    row_indices = rowstart:rowfinish;
    for j=1:nstates;
        colstart = (j-1)*disc_pts+1;
        colfinish = j*disc_pts;
        col_indices = colstart:colfinish;
        if isequal(dependencies(i+ndiffeqs,j),1)
            S(row_indices,col_indices) = Path_Block_State;
            Sjac(row_indices,col_indices) = Path_Block_State;
         else
            S(row_indices,col_indices) = Path_Block_State_Zero;
            Sjac(row_indices,col_indices) = Path_Block_State_Zero;
        end	    
    end;
    colshift = nstates*disc_pts;
    for j=1:ncontrols;
        colstart = colshift+(j-1)*nodes+1;
        colfinish = colshift+j*nodes;
        col_indices = colstart:colfinish;
        if isequal(dependencies(i+ndiffeqs,j+nstates),1)
            S(row_indices,col_indices) = Path_Block_Control;
            Sjac(row_indices,col_indices) = Path_Block_Control;
        else
            S(row_indices,col_indices) = Path_Block_Control_Zero;
            Sjac(row_indices,col_indices) = Path_Block_Control_Zero;
        end;
    end;
end;
rowshift = ndiffeqs*(nodes+1)+npaths*nodes;
event_indices_init = 1:nodes+2:ndiffeqs*(nodes+2);
event_indices_term = nodes+2:nodes+2:ndiffeqs*(nodes+2);
S(rowshift+1:rowshift+nevents,event_indices_init) = 1;
S(rowshift+1:rowshift+nevents,event_indices_term) = 1;
Sjac(rowshift+1:rowshift+nevents,event_indices_init) = 1;
Sjac(rowshift+1:rowshift+nevents,event_indices_term) = 1;
colshift = ndiffeqs*disc_pts+ncontrols*nodes;
S(:,colshift+1:colshift+2) = 1;
Sjac(:,colshift+1:colshift+2) = 1;
colshift = colshift+2;
S(:,colshift+1:colshift+nparameters) = 1;
Sjac(:,colshift+1:colshift+nparameters) = 1;
columns = ndiffeqs*disc_pts+ncontrols*nodes + 1 : ndiffeqs*disc_pts+ncontrols*nodes + 2 + nparameters;
for i = 1:ndiffeqs
    row = i*(nodes+1);
    S(row,columns) = 0;
    Sjac(row,columns) = 0;
end



