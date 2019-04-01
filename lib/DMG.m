function output= DMG(setup)

currdir = pwd;
%------------------------------------------------------------------%
%           Print Header to screen                                 %
%------------------------------------------------------------------%
disp('-----------------------------------------------------------------------------------');
disp('    ');
disp('   ____ ______   ____ ______  ______');
disp('  / ___\\____ \ /  _ \\____ \/  ___/');
disp(' / /_/  >  |_> >  <_> )  |_> >___ \ ');
disp(' \___  /|   __/ \____/|   __/____  >');
disp('/_____/ |__|          |__|       \/ ');
disp('      ');
disp(' Modified by DAVID MORANTE GONZALEZ (UPM) ');
disp('-----------------------------------------------------------------------------------');
disp('    ');
disp('-------------------------------------------------------------------------------------------');
disp('Objective Gradient Being Estimated via Built-In Automatic Differentiation');      
disp('-------------------------------------------------------------------------------------------');
disp('Constraint Jacobian Being Estimated via Built-In Automatic Differentiation');
disp('-------------------------------------------------------------------------------------------');

%------------------------------------------------------------------%
%        Check for the SOLVERS                                     %
%------------------------------------------------------------------%

if isfield(setup,'solver'),
    
    if isequal(setup.solver,'ipopt')
        clear ipopt      
        disp('-------------------------------------------------------------------------------------------');
        disp('Solving the Problem with the solver IPOPT'); 
        disp('-------------------------------------------------------------------------------------------');
        disp('    '); 
        
        if isempty(which('ipopt'))
        error('IPOPT not found or installed incorrectly')
        end

    elseif isequal(setup.solver,'snopt')
        clear snopt
        disp('-------------------------------------------------------------------------------------------');
        disp('Solving the Problem with the solver SNOPT'); 
        disp('-------------------------------------------------------------------------------------------');
        disp('    '); 
        
        if isempty(which('snopt'))
        error('SNOPT not found or installed incorrectly')
        end
        
    elseif isequal(setup.solver,'fmincon')
        clear fmincon
        disp('-------------------------------------------------------------------------------------------');
        disp('Solving the Problem with the solver FMINCON'); 
        disp('-------------------------------------------------------------------------------------------');
        disp('    '); 
        if isempty(which('fmincon'))
        error('FMINCON not found or installed incorrectly')
        end
    
    else
        
    error('A SOLVER must be selected. Use setup.solver=ipopt or setup.solver=snopt in the main file');
        
    end
else
    
    error('A SOLVER must be selected. Use setup.solver=ipopt or setup.solver=snopt in the main file');
    
end

%------------------------------------------------------------------%
%        Check for the METHODS                                     %
%------------------------------------------------------------------%

if isfield(setup,'method'),
    
    if isequal(setup.method,'collocation')
        disp('-------------------------------------------------------------------------------------------');
        disp('Applying Direct Collocation Method'); 
        disp('-------------------------------------------------------------------------------------------');
        disp('    '); 
        
        disp('-------------------------------------------------------------------------------------------');
        disp(' Direct Collocation Method Does not support Multiphase problems'); 
        disp('-------------------------------------------------------------------------------------------');
        disp('    '); 
        
            if isequal(setup.autoscale,'on')
                error('Collocation method does not support automatic scaling. Select setup.autoscale == offs')
            end
        output=gpops_coll(setup);


    elseif isequal(setup.method,'pseudospectral')
        disp('-------------------------------------------------------------------------------------------');
        disp('Applying Gauss Pseudospectral Method')
        disp('-------------------------------------------------------------------------------------------');
        disp('    '); 

        output=gpops_pseudo(setup);
    else
        
        error('A METHOD must be selected. Use setup.method=collocation or setup.method=pseudospectral in the main file')
        
    end
else
    
    error('A METHOD must be selected. Use setup.method=collocation or setup.method=pseudospectral in the main file');
    
end






