function output = dmg_pseudo(setup)
%------------------------------------------------------------------%
%       DMG:  Direct Transcription Method 
%       Using Gauss Pseudospectral Collocaiton Method
%------------------------------------------------------------------%
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
global mysetup;
clear snoptcmex;
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
%     Matrix containing coefficients of the linear constraints     %
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

if isfield(setup,'autoscale')
     if isequal(setup.autoscale,'on')
       
         xlow    = xlow.*setup.column_scales;
         xupp    = xupp.*setup.column_scales;
         clow    = clow.*setup.row_scales;
         cupp    = cupp.*setup.row_scales;
         init    = init.*setup.column_scales;

         if isequal(setup.solver,'snopt')
                Alinear = Alinear*diag(1./setup.column_scales); 
         end
         
         disp('Automatic Scaling Turned On');
         %-------------------------------------------------- %
         % Give Warning for infinite bounds with autoscaling %
         %-------------------------------------------------- %
         if ~isempty(find(isinf(xlow)|isinf(xupp),1))
             disp('WARNING!!! Automatic Scaling may not work with infinite bounds')
             disp(' ')
         end
     elseif isequal(setup.autoscale,'off')
        setup.column_scales = ones(size(setup.column_scales));
        setup.row_scales = ones(size(setup.row_scales));
        disp('Automatic Scaling Turned Off');
     else
       error('setup.autoscale not set correctly');
     end
 else
    setup.column_scales = ones(size(setup.column_scales));
    setup.row_scales = ones(size(setup.row_scales));
end
 
 
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
setup.Alinear_augmented = Alinear_augmented(2:end,:);
setup.sparsity = setup.sparsity_all(2:end,:)+sparse(Alinear_augmented(2:end,:));
[iAfun,jAvar,AA] = find(Alinear_augmented);
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

%------------------------------------------------------------------%
% Set the lower and upper limits on the constraints and objective  %
% function.  The following assumptions are made:                   %
%  Row 1 is the objective row                                      %
%  Rows 2 through numnonlin+1 are the nonlinear constraints        %
%  Rows numnonlin+2 to the end are the linear constraints          %
%------------------------------------------------------------------%

mysetup=setup;

if isfield(setup,'solver')
    
    if isequal(setup.solver,'ipopt')
%##########################################################################
% Script lines modified by David Morante González to make IPOPT available
% to GPOPS
%------------------------------------------------------------------%
%                   Solve the NLP using IPOPT                      %
%------------------------------------------------------------------%
options.cl = [clow;Alinmin];
options.cu = [cupp;Alinmax];
options.lb = xlow;
options.ub = xupp;
options.ipopt.hessian_approximation = 'limited-memory';

if isfield(setup,'derivatives')
    deropt = lower(setup.derivatives);
    if isequal(deropt,'automatic')
        %---------------------------------------------------%
        % Check whether Built-In automatic differentiation  %
        % is installed on the machine.                      %
        %---------------------------------------------------%
        if isempty(which('ad.m'))
            error('Built-in automatic differentiator not found or installed incorrectly')
        end
        funcs.gradient =@dmg_gradient_AD;
        funcs.jacobian =@dmg_jacobian_AD;
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Built-In Automatic Differentiation');
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Built-In Automatic Differentiation');
                disp('------------------------------------------------------------------');
        
    elseif isequal(deropt,'numerical')
        mysetup.hpert = 1e-8;
        mysetup.deltaxmat = mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        funcs.gradient =@dmg_gradient_FD;
        funcs.jacobian =@dmg_jacobian_FD;
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Finite Differencing');
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Finite Differencing');
                disp('------------------------------------------------------------------');
        
    elseif isequal(deropt,'complex')
        mysetup.hpert = 1e-20;
        mysetup.deltaxmat = sqrt(-1)*mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        funcs.gradient =@dmg_gradient_CS;
        funcs.jacobian =@dmg_jacobian_CS;
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Complex Differentiation');  
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Complex Differentiation');
                disp('------------------------------------------------------------------');
        
    elseif isequal(deropt,'analytic')
        funcs.gradient =@dmg_gradient_AN;
        funcs.jacobian =@dmg_jacobian_AN;
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Analytic Differentiation');
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Analytic Differentiation');
                disp('------------------------------------------------------------------');
        
    else
        error(['Unknown derivative option "%s" in setup.derivatives\n',...
            'Valid options are:',...
            '\n\tautomatic        \t-> built-in Automatic Differentiation',...
            '\n\tnumerical        \t-> Finite Differencing',...
            '\n\tcomplex          \t-> Complex-Step Differentiation',...
            '\n\tanalytic         \t-> User Defined Analytic Differentiation\n'],...
            setup.derivatives);
        
    end
end
      
funcs.constraints       = @dmg_constraints_ipopt;
funcs.jacobianstructure = @dmg_jacobianstructure;
funcs.objective         = @dmg_objective;
mysetup.constraints     = @dmg_constraints;
mysetup.objective       = @dmg_objective;


JAC = feval(funcs.jacobian,init,mysetup);

mysetup.sparsity = ones(size(JAC));



options.auxdata = mysetup;

[x,info]= ipopt_auxdata(init,funcs,options);

result.Fmul=info.lambda;

%------------------------------------------------------------------%
%               Record the IPOPT info result                       %
%------------------------------------------------------------------%
setup.IPOPT_info = info;

%##########################################################################
% Lines modified up to here
%##########################################################################


    elseif isequal(setup.solver,'snopt')
%------------------------------------------------------------------%
%                   Solve the NLP using SNOPT                      %
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

if isfield(setup,'derivatives')
    deropt = lower(setup.derivatives);
    if isequal(deropt,'automatic')
        %---------------------------------------------------%
        % Check whether Built-In automatic differentiation  %
        % is installed on the machine.                      %
        %---------------------------------------------------%
        if isempty(which('ad.m'))
            error('Built-in automatic differentiator not found or installed incorrectly')
        end
        userfun = 'gpopsuserfunAD';
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Built-In Automatic Differentiation');
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Built-In Automatic Differentiation');
                disp('------------------------------------------------------------------');
        
    elseif isequal(deropt,'numerical')
        mysetup.hpert = 1e-8;
        mysetup.deltaxmat = mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin+1,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        userfun = 'gpopsuserfunFD';
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Finite Differencing'); 
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Finite Differencing');
                disp('------------------------------------------------------------------');
        
    elseif isequal(deropt,'complex')
        mysetup.hpert = 1e-20;
        mysetup.deltaxmat = sqrt(-1)*mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin+1,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        userfun = 'gpopsuserfunCS';
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Complex Differentiation');    
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Complex Differentiation');
                disp('------------------------------------------------------------------');
        
    elseif isequal(deropt,'analytic')
        %-----------------------------------------------------------%
        % When using ANALYTIC DIFFERENTIATION, SNOPT will call the  %
        % function 'gpopsuserfunAN.m'                               %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunAN';
        snseti('Derivative Option',1);
                disp('------------------------------------------------------------------');
        disp('Objective Gradient Being Estimated via Analytic Differentiation');   
                disp('------------------------------------------------------------------');
        disp('Constraint Jacobian Being Estimated via Analytic Differentiation');
                disp('------------------------------------------------------------------');
    else
        error(['Unknown derivative option "%s" in setup.derivatives\n',...
            'Valid options are:',...
            '\n\tautomatic        \t-> built-in Automatic Differentiation',...
            '\n\tnumerical        \t-> Finite Differencing',...
            '\n\tcomplex          \t-> Complex-Step Differentiation',...
            '\n\tanalytic         \t-> User Defined Analytic Differentiation\n'],...
            setup.derivatives);
    end
end

mysetup.gpopsObjandCons = @gpopsObjandCons;

snscreen on
snset('LU Complete Pivoting');    % Choose Pivoting Method
snseti('Timing level',3);         % Print Execution Time to File

  [x,F,xmul,Fmul,info,xstate,Fstate,ns,ninf,sinf,mincw,miniw,minrw]...
      = snsolve(init,xlow,xupp,xmul,xstate,Flow,Fupp,Fmul,Fstate,...
               ObjAdd,ObjRow,AA,iAfun,jAvar,iGfun,jGvar,userfun);
snprint off;
result.Fmul=Fmul;
%------------------------------------------------------------------%
%               Record the SNOPT info result                       %
%------------------------------------------------------------------%
setup.SNOPT_info = info;

    end
else
    error('A solver must be selected by setup.solver=ipopt or setup.solver=snopt');
end

%------------------------------------------------------------------%
%                Unscale the NLP (Decision) Variables              %
%------------------------------------------------------------------%

result.x = x./setup.column_scales;

setup = mysetup;
setup.result = result;
%
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






