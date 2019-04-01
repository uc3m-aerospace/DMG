% ---------------------------------------------------------------- %
% This script runs all of the exampes in % the GPOPS distribution. %
% This script is  designed for testing to make sure that you have  %
% properly installed GPOPS.                                        %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, and Christopher Darby.                %
%------------------------------------------------------------------%

gpopsexamples = pwd;
j = 1;
directory{j} = strcat(gpopsexamples,'/brachistochrone/');
wrapperfile{j}  = 'brachistochroneWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/brysonDenham/');
wrapperfile{j}  = 'brysonDenhamWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/brysonMaxrange/');
wrapperfile{j}  = 'brysonMaxrangeWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/chemicalProcess/');
wrapperfile{j}  = 'chemicalProcessWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/goddardRocket/');
wrapperfile{j}  = 'goddardRocketWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/hyperSensitive/');
wrapperfile{j}  = 'hyperSensitiveWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/launch/');
wrapperfile{j}  = 'launchWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/leeBioreactor/');
wrapperfile{j}  = 'leeBioreactorWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/linearTangentSteering/');
wrapperfile{j}  = 'linearTangentWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/minimumClimb/');
wrapperfile{j}  = 'minimumClimbWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/moonLander/');
wrapperfile{j}  = 'moonlanderWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/rlvEntry/');
wrapperfile{j}  = 'rlvEntryWrapper';
j = j+1;
directory{j} = strcat(gpopsexamples,'/robotArm/');
wrapperfile{j}  = 'robotArmWrapper';
for i=1:length(directory);
    cd(directory{i});
    clear setup limits guess linkages solinit connections
    examplename = wrapperfile{i}(1:end-7);
    examplestr = strcat(['Running ',examplename,' Example']);
    disp(' ');
    disp(' ');
    disp(examplestr);
    disp(' ');
    disp(' ');
    pause(1);
    feval(wrapperfile{i});
    gpopsClean;
end;
cd(gpopsexamples);    
