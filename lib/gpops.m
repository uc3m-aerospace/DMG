function output = gpops(setup);
%------------------------------------------------------------------%
%       GPOPS:  Gauss Pseudopsectral Optimal Control Program       %
%------------------------------------------------------------------%
%  GPOPS is a MATLAB(R) program for solving non-sequential         %
%  multiple-phase optimal control problems. GPOPS uses the Gauss   %
%  pseudospectral method (GPM) where orthogonal collocation is     %
%  performed at the Legendre-Gauss points.  GPOPS is based         %
%  entirely on mathematical theory that has been published in the  %
%  open literature.  Specifically, a good portion of the theory    %
%  of the Gauss pseudospectral method can be found in the          %
%  publications:                                                   %
%                                                                  %
%  [1] Benson, D. A., Huntington, G. T., Thorvaldsen, T. P., and   %
%      Rao, A. V., "Direct Trajectory Optimization and Costate     %
%      Estimation via an Orthogonal Collocation Method," Journal   %
%      of Guidance, Control, and Dynamics, Vol. 29, No. 6,         %
%      November-December 2006, pp. 1435-1440.                      %
%                                                                  %
%  [2] Benson, D. A., "A Gauss Pseudospectral Transcription for    %
%      Optimal Control," Ph.D. Thesis, Dept. of Aeronautics and    %
%      Astronautics, Massachusetts Institute of Technology,        %
%      February  2005.                                             %
%                                                                  %  
%  [3] Huntington, G. T., "Advancement and Analysis of a Gauss     %
%      Pseudospectral Transcription for Optimal Control,"          %
%      Ph.D. Thesis, Dept. of Aeronautics and Astronautics,        $
%      Massachusetts Institute of Technology, May 2007.            %
%                                                                  %
%  [4] Rao, A. V., Benson, D. A., Huntington, G. T., Francolin, C. %
%      Darby, C. L., and Patterson, M. A., "User's Manual for      %
%      GPOPS:  A MATLAB Package for Dynamic Optimization Using the %
%      Gauss Pseudospectral Method, University of Florida, Report, %
%      August 2008.                                                %
%                                                                  %
%  Further information about the pseudoespectral theory used in    %
%  GPOPS and the applications of this theory to various problems   %
%  of interest can be found at Anil V. Rao's website at            %
%  http://fdol.mae.ufl.edu.                                        %
%                                                                  %
%------------------------------------------------------------------%
%  Authors of the GPOPS Code:                                      %
%    David A. Benson:      Draper Laboratory, Cambridge, MA        %
%    Geoffrey Huntington:  Blue Origin, LLC, Seattle, WA           %
%    Michael Patterson:    University of Florida, Gainesville, FL  %
%    Christopher Darby:    University of Florida, Gainesville, FL  %
%    Camila Francolin:     University of Florida, Gainesville, FL  %
%    Anil V. Rao:          University of Florida, Gainesville, FL  %
%  File creation date:  5 August 2008                              %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
% For Licencing Information, Please See File LICENSE               %
% ---------------------------------------------------------------- %
%                                                                  %
%                SEE THE GPOPS MANUAL FOR USAGE                    %
%                                                                  %
%------------------------------------------------------------------%

%------------------------------------------------------------------%
% SNOPTA does not allow for variable input arguments.              %
% Consequently, a copy needs to be make of the master structure    %
% SETUP. This structure needs to be made a global variable for use %
% the various functions.                                           %
%------------------------------------------------------------------%
global mysetup;
%------------------------------------------------------------------%
%           Print Header to screen                                 %
%------------------------------------------------------------------%
disp('    ');
disp('   ____ ______   ____ ______  ______');
disp('  / ___\\____ \ /  _ \\____ \/  ___/');
disp(' / /_/  >  |_> >  <_> )  |_> >___ \ ');
disp(' \___  /|   __/ \____/|   __/____  >');
disp('/_____/ |__|          |__|       \/ ');
disp('      ');
disp('-----------------------------------------------------------------------------------');
disp('GPOPS Version 2.2 beta: A MATLAB Implementation of the Gauss Pseudospectral Method');
disp('   Copyright (c) 2008 Anil V. Rao, David Benson, Geoffrey T. Huntington,  ')
disp('   Christopher L. Darby, Michael Patterson, and Camila Francolin');
disp('-----------------------------------------------------------------------------------');
disp('    ');
disp('-------------------------------------------------------------------------------------------');
disp('Downloading, using, copying, or modifying the GPOPS code constitutes an agreement to ALL of');
disp('the terms of the GPOPS license. Please see the file LICENSE given in the home directory of');
disp('the GPOPS distribution or see the summary file printed when running an example.');
disp('-------------------------------------------------------------------------------------------');
disp('    '); 
%------------------------------------------------------------------%
%           Clear any previous calls to the SNOPT mex file         %
%------------------------------------------------------------------%
clear snoptcmex;
%------------------------------------------------------------------%
%        Check for SNOPT installation                              %
%------------------------------------------------------------------%
if isempty(which('snoptcmex'))
    error('SNOPT not found or installed incorrectly')
end
%------------------------------------------------------------------%
%    Check input for backwards compatability                       %
%------------------------------------------------------------------%
setup = gpopsConvertInput(setup);
%------------------------------------------------------------------%
%    Get the sizes in each phase of the optimal control problem    %
%------------------------------------------------------------------%
setup = gpopsGetSizes(setup);
%------------------------------------------------------------------%
%    Get the bounds in each phase of the optimal control problem   %
%------------------------------------------------------------------%
setup = gpopsGetBounds(setup);
%------------------------------------------------------------------%
%        Print a description of the optimal control problem        %
%------------------------------------------------------------------%
gpopsPrint(setup);
%------------------------------------------------------------------%
%               Get the initial guess for the NLP                  %
%------------------------------------------------------------------%
setup = gpopsGetGuess(setup); 
%------------------------------------------------------------------%
%                  Scale the nonlinear program                     %
%------------------------------------------------------------------%
setup = gpopsScaleNlp(setup);
%------------------------------------------------------------------%
%            Determine the sparsity pattern of the NLP             %
%------------------------------------------------------------------%
setup = gpopsSparsity(setup);
%------------------------------------------------------------------%
%           Lower and upper bounds on the NLP variables            %
%------------------------------------------------------------------%
xlow = setup.varbounds_min;
xupp = setup.varbounds_max;
%------------------------------------------------------------------%
%       Lower and upper bounds on the nonlinear constraints        %
%------------------------------------------------------------------%
clow = setup.conbounds_min;
cupp = setup.conbounds_max;
%------------------------------------------------------------------%
%                   Initial guess for the NLP                      %
%------------------------------------------------------------------%
init = setup.init_vector;
%------------------------------------------------------------------%
%     Matrix containing coefficients of the linear constraints  
%Necesario para incluir que t_f-t0>0%
%------------------------------------------------------------------%
Alinear = setup.Alinear;
%------------------------------------------------------------------%
%        Lower and upper bounds on the linear constraints          %
%------------------------------------------------------------------%
Alinmin = setup.Alinmin;
Alinmax = setup.Alinmax;
%------------------------------------------------------------------%
%  Modify the lower & upper bounds on the variables & constraints  %
%  depending upon whether or not the user has chosen to            %
%  automatically scale the NLP.                                    %
%------------------------------------------------------------------%
if isfield(setup,'autoscale'),
    if isequal(setup.autoscale,'on'),
        xlow = xlow.*setup.column_scales;
        xupp = xupp.*setup.column_scales;
        clow = clow.*setup.row_scales;
        cupp = cupp.*setup.row_scales;
        init   = init.*setup.column_scales;
        Alinear = Alinear*diag(1./setup.column_scales);
        disp('Automatic Scaling Turned On');
        %-------------------------------------------------- %
        % Give Warning for infinite bounds with autoscaling %
        %-------------------------------------------------- %
        if ~isempty(find(isinf(xlow)|isinf(xupp),1))
            disp('WARNING!!! Automatic Scaling may not work with infinite bounds')
            disp(' ')
        end
    elseif isequal(setup.autoscale,'off'),
        setup.column_scales = ones(size(setup.column_scales));
        setup.row_scales = ones(size(setup.row_scales));
        disp('Automatic Scaling Turned Off');
    else
      error('setup.autoscale not set correctly');
    end;
else
    setup.column_scales = ones(size(setup.column_scales));
    setup.row_scales = ones(size(setup.row_scales));
end;
Alinear = sparse(Alinear);
setup.Alinear = Alinear;
%------------------------------------------------------------------%
% Setup diagonal matrices containing the scale factors computed by %
% the automatic scaling routine.                                   %
%------------------------------------------------------------------%
setup.Dx = sparse(diag(setup.column_scales));
setup.invDx = sparse(diag(1./setup.column_scales));
setup.DF = sparse(diag(setup.row_scales));
setup.invDF = sparse(diag(1./setup.row_scales));
%------------------------------------------------------------------%
%             Find the row and column indices in S_ALL             %
%------------------------------------------------------------------%
[iGfun,jGvar]=find(setup.sparsity_all);  
%------------------------------------------------------------------%
%             Sort the row and column indices by rows.             %
%------------------------------------------------------------------%
JJ = sortrows([iGfun jGvar],1); 
iGfun = JJ(:,1);
jGvar = JJ(:,2);
setup.iGfun = iGfun;
setup.jGvar = jGvar;
%------------------------------------------------------------------%
%            SCONSTANT constains the CONSTANT derivatives          %
%------------------------------------------------------------------%
Sconstant = setup.DF*setup.sparsity_constant*setup.invDx;
%------------------------------------------------------------------%
% ALINEAR_AUGMENTED is a matrix of the linear constraints plus the %
% constant derivatives.                                            %
%------------------------------------------------------------------%
Alinear_augmented = [zeros(1,setup.numvars); Sconstant; Alinear];
[iAfun,jAvar,AA] = find(Alinear_augmented);
mysetup.A=Alinear_augmented;
%------------------------------------------------------------------%
%             Sort the row and column indices by rows              %
%------------------------------------------------------------------%
II = sortrows([iAfun jAvar AA],1);
iAfun = II(:,1);
jAvar = II(:,2);
AA    = II(:,3);
%------------------------------------------------------------------%
%        Free MATLAB memory by clearing unneeded variables         %
%------------------------------------------------------------------%
clear JJ II Sconstant Alinear Alinear_augmented
%------------------------------------------------------------------%
% This section sets the appropriate differentiation                %
% method for use with the NLP solver.                              %
%------------------------------------------------------------------%
if isfield(setup,'derivatives'),
    deropt = lower(setup.derivatives);
    if isequal(deropt,'automatic')
        %---------------------------------------------------%
        % Check whether Built-In automatic differentiation  %
        % is installed on the machine.                      %
        %---------------------------------------------------%
        if isempty(which('ad.m'))
            error('Built-in automatic differentiator not found or installed incorrectly')
        end
        %-----------------------------------------------------------%
        % If setup.derivatives equals 'automatic', then use the     %
        % built-in automatic differentiator for the computation of  %
        % derivatives.  When using BUILT-IN AUTOMATIC               %
        % DIFFERENTIATION, SNOPT will call the function             %
        % 'gpopsuserfunAD.                                          %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunAD';
        snseti('Derivative Option',1);
        disp('Objective Gradient Being Estimated via Built-In Automatic Differentiation');        
        disp('Constraint Jacobian Being Estimated via Built-In Automatic Differentiation');
    elseif isequal(deropt,'automatic-mad'),
        %---------------------------------------------------%
        % Check whether MATLAB automatic differentiation    %
        % is installed on the machine.                      %
        %---------------------------------------------------%
        if isempty(which('checkMAD.m'))
            error('MAD not found or installed incorrectly')
        end
        if ~checkMAD(0),
            madinitglobals;
        end;
        %---------------------------------------------------%
        % Set up a bunch of stuff related to MAD when using %
        % automatic differentiation                         %
        %---------------------------------------------------%
        color_groups = MADcolor(setup.sparsity_all);
        setup.seed = sparse(MADgetseed(setup.sparsity_all,color_groups));
        setup.color_groups = color_groups;
        %-----------------------------------------------------------%
        % When using AUTOMATIC DIFFERENTIATION, SNOPT will call the %
        % function 'gpopsuserfunADMAD.m'                            %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunADMAD';
        snseti('Derivative Option',1);
        disp('Objective Gradient Being Estimated via Matlab Automatic Differentiation (MAD)');        
        disp('Constraint Jacobian Being Estimated via Matlab Automatic Differentiation (MAD)');
        disp('   WARNING, MAD is not supported in GPOPS and may not continue to function');
        disp('   in future versions of GPOPS and/or MAD.')
    elseif isequal(deropt,'automatic-intlab')
        %---------------------------------------------------%
        % Check whether INTLAB automatic differentiation    %
        % is installed on the machine.                      %
        %---------------------------------------------------%
        if isempty(which('gradientinit.m'))
            error('INTLAB not found or installed incorrectly')
        end
        %-----------------------------------------------------------%
        % If setup.derivatives equals EITHER 'automatic-INTLAB'     %
        % OR 'automatic', then use INTLAB for computing derivatives %
        % When using INTLAB AUTOMATIC DIFFERENTIATION, SNOPT will   %
        % function 'gpopsuserfunADINT.m'                            %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunADINT';
        snseti('Derivative Option',1);
        disp('Objective Gradient Being Estimated via INTLAB Automatic Differentiation');        
        disp('Constraint Jacobian Being Estimated via INTLAB Automatic Differentiation');
    elseif isequal(deropt,'numerical'),
        %-----------------------------------------------------------%
        % When using NUMERICAL DIFFERENTIATION, SNOPT will call the %
        % function 'gpopsuserfunFD.m'                               %
        %-----------------------------------------------------------% 
        userfun = 'gpopsuserfunFD';
        snseti('Derivative Option',0);
        disp('Objective Gradient Being Estimated via Finite Differencing');        
        disp('Constraint Jacobian Being Estimated via Finite Differencing');
    elseif isequal(deropt,'complex'),
        %-----------------------------------------------------------%
        % When using COMPLEX-STEP DIFFERENTIATION, SNOPT will call  %
        % the function 'gpopsuserfunCS.m'                           %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunCS';
        snseti('Derivative Option',1);
        setup.hpert = 1e-20;
        setup.deltaxmat = sqrt(-1)*setup.hpert*speye(setup.numvars);
        setup.Jaczeros = zeros(setup.numnonlin+setup.numlin+1,setup.numvars);
        disp('Objective Gradient Being Estimated via Complex Differentiation');        
        disp('Constraint Jacobian Being Estimated via Complex Differentiation');
    elseif isequal(deropt,'analytic'),
        %-----------------------------------------------------------%
        % When using ANALYTIC DIFFERENTIATION, SNOPT will call the  %
        % function 'gpopsuserfunAN.m'                               %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunAN';
        snseti('Derivative Option',1);
        disp('Objective Gradient Being Estimated via Analytic Differentiation');        
        disp('Constraint Jacobian Being Estimated via Analytic Differentiation');
        if isfield(setup,'checkDerivatives') && setup.checkDerivatives == 1
            %---------------------------------%
            % Check user supplied derivatives %
            %---------------------------------%
            gpopsCheckDerivatives(setup);
        end
    else
        error(['Unknown derivative option "%s" in setup.derivatives\n',...
            'Valid options are:',...
            '\n\tautomatic        \t-> GPOPS built-in Automatic Differentiation',...
            '\n\tautomatic-INTLAB \t-> INTLAB Automatic Differentiation',...
            '\n\tautomatic-MAD    \t-> Matlab Automatic Differentiation'...
            '\n\tnumerical        \t-> SNOPT Finite Differencing',...
            '\n\tcomplex          \t-> Complex-Step Differentiation',...
            '\n\tanalytic         \t-> User Defined Analytic Differentiation\n'],...
            setup.derivatives);
    end;
else
    %-----------------------------------------------------------%
    % When using NUMERICAL DIFFERENTIATION, SNOPT will call the %
    % function 'gpopsuserfunFD.m'                               %
    %-----------------------------------------------------------%
    userfun = 'gpopsuserfunFD';
    snseti('Derivative Option',0);
    disp('Objective Gradient Being Computed via Finite Differencing');
    disp('Constraint Jacobian Being Computed via Finite Differencing');
end;
%---------------------------------------%
% NUMLIN = Number of linear constraints %
%---------------------------------------%
numlin    = setup.numlin;
%------------------------------------------%
% NUMNONLIN = Number of linear constraints %
%------------------------------------------%
numnonlin = setup.numnonlin;
%---------------------------------------------------------------%
% Pre-allocate a bunch of vectors for use in the user function. %
% This pre-allocation is done to reduce execution time.         %
%---------------------------------------------------------------%
setup.initlincons = zeros(numlin,size(init,2));
%----------------------------------------------------------%
% Set up settings that are used by SNOPT for every problem %
%----------------------------------------------------------%
snprint('snoptmain.out');         % Name of SNOPT Print File
snseti('Timing level',3);         % Print Execution Time to File
snset('Hessian Limited Memory');  % Choose Hessian Type
snset('LU Complete Pivoting');    % Choose Pivoting Method
snseti('Verify Level',-1);        % Derivative Verification Level
if isfield(setup,'maxIterations'),
    if isa(setup.maxIterations,'double'),
        maxIters = setup.maxIterations;
    else
        error('field <maxIterations> must be a double');
    end;
else
    maxIters = 100000;
end;
snseti('Iteration Limit',10*maxIters); % Iteration Limit
snseti('Major Iterations Limit',maxIters); % Major Iteration Limit
snseti('Minor Iterations Limit',maxIters); % Minor Iteration Limit
if isfield(setup,'tolerances');
    if isequal(numel(setup.tolerances),2),
        if ~isempty(setup.tolerances(1)),
            snsetr('Optimality Tolerance',setup.tolerances(1));
            snsetr('Major feasibility Tolerance',setup.tolerances(2));
        else
            snsetr('Optimality Tolerance',setup.tolerances(2));
        end;
    elseif isequal(numel(setup.tolerances),1),
        snsetr('Optimality Tolerance',setup.tolerances(1));
    end;
end;
%------------------------------------------------------------------%
% Set the lower and upper limits on the constraints and objective  %
% function.  The following assumptions are made:                   %
%  Row 1 is the objective row                                      %
%  Rows 2 through numnonlin+1 are the nonlinear constraints        %
%  Rows numnonlin+2 to the end are the linear constraints          %
%------------------------------------------------------------------%
Flow = [-Inf; clow; Alinmin];
Fupp = [ Inf; cupp; Alinmax];
%-----------------------------------------------------------------%
%  Initialize the Lagrange multipliers on the constraints to zero  %
%------------------------------------------------------------------%
Fmul = zeros(numnonlin+numlin+1,1);
Fstate = Fmul;
%------------------------------------------------------------------%
%   Initialize the Lagrange multipliers on the variables to zero   %
%------------------------------------------------------------------%
xmul = zeros(setup.numvars,1);
xstate = xmul;
%------------------------------------------------------------------%
%       Objrow =1 <=======> First row is the objective row         %
%------------------------------------------------------------------%
ObjRow = 1;
%------------------------------------------------------------------%
%  ObjAdd =0 <====> Do not add anything to the objective function  %
%------------------------------------------------------------------%
ObjAdd = 0;
%------------------------------------------------------------------%
%                     Turn on screen output                        %
%------------------------------------------------------------------%
snscreen on
%------------------------------------------------------------------%
%  Set MYSETUP equal to SETUP for global use in the optimization.  %
%------------------------------------------------------------------%
mysetup = setup;
%------------------------------------------------------------------%
%                   Solve the NLP using SNOPT                      %
%------------------------------------------------------------------%
[x,F,xmul,Fmul,info,xstate,Fstate,ns,ninf,sinf,mincw,miniw,minrw]...
    = snsolve(init,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,...
             ObjAdd,ObjRow,AA,iAfun,jAvar,iGfun,jGvar,userfun);
snprint off;
%------------------------------------------------------------------%
%                Unscale the NLP (Decision) Variables              %
%------------------------------------------------------------------%
result.x = x./setup.column_scales;
%------------------------------------------------------------------%
%              Unscale the multipliers on the variables            %
%------------------------------------------------------------------%
result.xmul = setup.Dx*xmul;
%------------------------------------------------------------------%
%            Unscale the multipliers on the constraints            %
%------------------------------------------------------------------%
result.Fmul = setup.DF*Fmul(2:numnonlin+1);
%------------------------------------------------------------------%
%         Extract the multipliers on the linear constraints        %
%------------------------------------------------------------------%
result.Amul = Fmul(numnonlin+2:end);
setup = mysetup;
setup.result = result;
%------------------------------------------------------------------%
%               Record the SNOPT info result                       %
%------------------------------------------------------------------%
setup.SNOPT_info = info;
%------------------------------------------------------------------%
%         Untranscribe the NLP (i.e., convert the optimal          %
%         control problem back to optimal control format)          %
%------------------------------------------------------------------%
setup = gpopsNlp2oc(setup);
%------------------------------------------------------------------%
%         Clear fields in the SETUP structure that are not         %
%         needed by the user (i.e., remove all fields that         %
%         were only used for internal calculations)                %
%------------------------------------------------------------------%
setup = gpopsClearFields(setup);
%------------------------------------------------------------------%
%    Set output for backwards compatability                        %
%------------------------------------------------------------------%
setup = gpopsConvertOutput(setup);
output = setup;
