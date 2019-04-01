function output = gpops_coll(setup)
%------------------------------------------------------------------%
%       DMOC:  Direct Transcription Method 
%       Using Third-degree Hermite interpolant
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
setup = gpopsGetBounds_coll(setup);
%------------------------------------------------------------------%
%        Print a description of the optimal control problem        %
%------------------------------------------------------------------%
gpopsPrint(setup);
%------------------------------------------------------------------%
%               Get the initial guess for the NLP                  %
%------------------------------------------------------------------%
setup = gpopsGetGuess_coll(setup); 
%------------------------------------------------------------------%
%                  Scale the nonlinear program                     %
%------------------------------------------------------------------%
setup = gpopsScaleNlp_coll(setup);
%------------------------------------------------------------------%
%            Determine the sparsity pattern of the NLP             %
%------------------------------------------------------------------%
%setup = gpopsSparsity(setup);
%------------------------------------------------------------------%
%           Lower and upper bounds on the NLP variables            %
%------------------------------------------------------------------%
xlow = setup.varbounds_min;
xupp = setup.varbounds_max;
setup.variables = numel(xlow);
%------------------------------------------------------------------%
%       Lower and upper bounds on the nonlinear constraints        %
%------------------------------------------------------------------%
clow = setup.conbounds_min;
cupp = setup.conbounds_max;
setup.constraints=numel(clow)+1;
%------------------------------------------------------------------%
%                   Initial guess for the NLP                      %
%------------------------------------------------------------------%
init = setup.init_vector;
%------------------------------------------------------------------%
%  Modify the lower & upper bounds on the variables & constraints  %
%  depending upon whether or not the user has chosen to            %
%  automatically scale the NLP.                                    %
%------------------------------------------------------------------%
if isfield(setup,'autoscale')
    if isequal(setup.autoscale,'on')
        xlow = xlow.*setup.column_scales;
        xupp = xupp.*setup.column_scales;
        clow = clow.*setup.row_scales;
        cupp = cupp.*setup.row_scales;
        init   = init.*setup.column_scales;
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
%Alinear = sparse(Alinear);
%setup.Alinear = Alinear;
%------------------------------------------------------------------%
% Setup diagonal matrices containing the scale factors computed by %
% the automatic scaling routine.                                   %
%------------------------------------------------------------------%
setup.Dx = sparse(diag(setup.column_scales));
setup.invDx = sparse(diag(1./setup.column_scales));
setup.DF = sparse(diag(setup.row_scales));
setup.invDF = sparse(diag(1./setup.row_scales));
setup.Alinear_augmented = [zeros(numel(clow),setup.numvars); setup.Alinear];
setup.sparsity = ones(size(setup.Alinear_augmented));
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


if isfield(setup,'solver'),
    
    if isequal(setup.solver,'ipopt')
%------------------------------------------------------------------%
%                   Solve the NLP using IPOPT                      %
%------------------------------------------------------------------%

options.cl = [clow;setup.Alinmin];
options.cu = [cupp;setup.Alinmax];
options.lb = xlow;
options.ub = xupp;
%options.ipopt.mu_strategy      = 'adaptive';
%options.ipopt.nlp_scaling_method='gradient-based';
%options.ipopt.nlp_scaling_max_gradient= 1;
%options.ipopt.bound_mult_init_val = 0.01;
%options.ipopt.nlp_scaling_min_value = 1e-1;
options.ipopt.tol=1e-6;
options.ipopt.bound_push = 1e-6;
options.ipopt.mu_init= 1e-2;
%options.ipopt.max_iter = 0;
%options.dual_inf_tol    = 0.1;
options.ipopt.hessian_approximation = 'limited-memory';

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
        funcs.gradient =@gradient_AD;
        funcs.jacobian =@jacobian_AD;
        disp('Objective Gradient Being Estimated via Built-In Automatic Differentiation');        
        disp('Constraint Jacobian Being Estimated via Built-In Automatic Differentiation');
        
    elseif isequal(deropt,'numerical'),
        mysetup.hpert = 1e-7;
        mysetup.deltaxmat = mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        funcs.gradient =@gradient_FD;
        funcs.jacobian =@jacobian_FD;
        disp('Objective Gradient Being Estimated via Finite Differencing');        
        disp('Constraint Jacobian Being Estimated via Finite Differencing');
        
    elseif isequal(deropt,'complex'),
        mysetup.hpert = 1e-20;
        mysetup.deltaxmat = sqrt(-1)*mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        funcs.gradient =@gradient_CS;
        funcs.jacobian =@jacobian_CS;
        disp('Objective Gradient Being Estimated via Complex Differentiation');        
        disp('Constraint Jacobian Being Estimated via Complex Differentiation');
        
    elseif isequal(deropt,'analytic'),
        funcs.gradient =@gradient_AN;
        funcs.jacobian =@jacobian_AN;
        disp('Objective Gradient Being Estimated via Analytic Differentiation');
        disp('Constraint Jacobian Being Estimated via Analytic Differentiation');
        
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

mysetup.Alinear_augmented = zeros(setup.numnonlin+setup.numlin,setup.numvars);
funcs.constraints =@constraints_coll;
funcs.jacobianstructure =@jacobianstructure_coll;
funcs.objective =@objective_coll;

mysetup.constraints = @constraints_coll;
mysetup.objective   = @objective_coll;


options.auxdata = mysetup;

[x,info]= ipopt_auxdata(init,funcs,options);

    elseif isequal(setup.solver,'snopt')
%------------------------------------------------------------------%
%                   Solve the NLP using SNOPT                      %
%------------------------------------------------------------------%
Flow = [-Inf; clow; setup.Alinmin];
Fupp = [ Inf; cupp; setup.Alinmax];
snscreen on

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
        userfun = 'gpopsuserfunAD';
        disp('Objective Gradient Being Estimated via Built-In Automatic Differentiation');        
        disp('Constraint Jacobian Being Estimated via Built-In Automatic Differentiation');
        
    elseif isequal(deropt,'numerical')
        mysetup.hpert = 1e-8;
        mysetup.deltaxmat = mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin+1,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        userfun = 'gpopsuserfunFD';
        disp('Objective Gradient Being Estimated via Finite Differencing');        
        disp('Constraint Jacobian Being Estimated via Finite Differencing');
        
    elseif isequal(deropt,'complex')
        mysetup.hpert = 1e-20;
        mysetup.deltaxmat = sqrt(-1)*mysetup.hpert*speye(setup.numvars);
        mysetup.Jaczeros = zeros(setup.numnonlin+setup.numlin+1,setup.numvars);
        mysetup.Gzeros   = zeros(1,setup.numvars);
        userfun = 'gpopsuserfunCS';
        disp('Objective Gradient Being Estimated via Complex Differentiation');        
        disp('Constraint Jacobian Being Estimated via Complex Differentiation');
        
            elseif isequal(deropt,'analytic')
        %-----------------------------------------------------------%
        % When using ANALYTIC DIFFERENTIATION, SNOPT will call the  %
        % function 'gpopsuserfunAN.m'                               %
        %-----------------------------------------------------------%
        userfun = 'gpopsuserfunAN';
        snseti('Derivative Option',1);
        disp('Objective Gradient Being Estimated via Analytic Differentiation');
        disp('Constraint Jacobian Being Estimated via Analytic Differentiation');
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

mysetup.gpopsObjandCons = @gpopsObjandCons_coll;

[x,F]= snopt(init,xlow,xupp,Flow,Fupp,userfun);

snprint off;

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

%------------------------------------------------------------------%
%               Record the SNOPT info result                       %
%------------------------------------------------------------------%
%setup.SNOPT_info = info;
%------------------------------------------------------------------%
%         Untranscribe the NLP (i.e., convert the optimal          %
%         control problem back to optimal control format)          %
%------------------------------------------------------------------%
setup = gpopsNlp2oc_coll(setup);
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
%
output = setup;





