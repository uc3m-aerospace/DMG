function output= DMG(setup)
%------------------------------------------------------------------%
%           Print Header to screen                                 %
%------------------------------------------------------------------%
disp('-----------------------------------------------------------------------------------');
disp('  _____   __    __   _____ ');
disp(' |  _  \ |  \  /  | |  ___|');
disp(' | |  \ \|   \/   | | | __ ');
disp(' | |_ / /|  /\/\  | | |_< |  ');
disp(' |_____/ |_|    |_| |_____|   ');
disp('-----------------------------------------------------------------------------------');
disp(' A Matlab General Purpose Tool for solving Multi-phase Optimal Control Problems  ');
disp(' derived from GPOPS ');
disp('-----------------------------------------------------------------------------------');
disp('    ');
disp('    ');
disp('-------------------------------------------------------------------------------------------');
disp('Downloading, using, copying, or modifying the DMG code constitutes an agreement to ALL of');
disp('the terms of the DMG license. Please see the file LICENSE given in the home directory of');
disp('the DMG distribution or see the file printed when running a particular example.');
disp('-------------------------------------------------------------------------------------------');
disp('    ');

%------------------------------------------------------------------%
%        Check for the SOLVERS                                     %
%------------------------------------------------------------------%

if isfield(setup,'solver')
    
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
        
    else
        
        error('A SOLVER must be selected. Use setup.solver=ipopt or setup.solver=snopt in the main file');
        
    end
else
    
    error('A SOLVER must be selected. Use setup.solver=ipopt or setup.solver=snopt in the main file');
    
end

%------------------------------------------------------------------%
%        Check for the METHODS                                     %
%------------------------------------------------------------------%

if isfield(setup,'method')
    
    if isequal(setup.method,'collocation')
        disp('-------------------------------------------------------------------------------------------');
        disp('Applying Hermite Simpson Collocation Method');
        disp('-------------------------------------------------------------------------------------------');
        disp('    ');
        
        if isequal(setup.autoscale,'on')
            error('Collocation method does not support automatic scaling. Select setup.autoscale == offs')
        end
        output=dmg_coll(setup);
        
        
    elseif isequal(setup.method,'pseudospectral')
        disp('-------------------------------------------------------------------------------------------');
        disp('Applying Gauss Pseudospectral Collocation Method')
        disp('-------------------------------------------------------------------------------------------');
        disp('    ');
        
        output=dmg_pseudo(setup);
    else
        
        error('A METHOD must be selected. Select setup.method=collocation or setup.method=pseudospectral in the main file')
        
    end
else
    
    error('A METHOD must be selected. Use setup.method=collocation or setup.method=pseudospectral in the main file');
    
end






