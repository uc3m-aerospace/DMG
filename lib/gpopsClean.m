function gpopsClean;
%------------------------------------------------------------------%
% This function cleans all output files from a run of GPOPS.       %
% The files cleaned by this function are as follows:               %
%   (1)  The main snopt output file:  snoptmain.out                %
%   (2)  The endpoint control snopt output files:                  %
%        snoptmain0.out, snoptmainF.out                            %
%   (3)  The text file with the problem statement                  %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

% delete text file 
if length(dir('*.txt')) > 1
    fprintf('gpopsClean Warning: more than one .txt file in directory\n')
else
    delete('*.txt');
end
% delete snopt output files
if ~isempty(dir('snoptmain.out'))
    delete('snoptmain.out');
end;
if ~isempty(dir('snoptmain0.out'))
    delete('snoptmain0.out');
end;
if ~isempty(dir('snoptmainF.out'))
    delete('snoptmainF.out');
end;

