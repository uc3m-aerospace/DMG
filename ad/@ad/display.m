function display(obj);

% AD implementation of display.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

warning off

disp('   ');
disp(sprintf('%s Object',upper(class(obj))));
disp('   ');
disp(sprintf('Value(s) of %s Object:',upper(class(obj))));
disp(obj.value);
sizeValue = size(obj.value);
sizeDerivative = [sizeValue obj.nderivs];
disp('   ');
disp(sprintf('Derivative(s) of %s Object:',upper(class(obj))));
sizeDerivative
disp(reshape(obj.derivative,sizeDerivative));
disp('   ');
disp('Number of Derivatives:');
disp(obj.nderivs);

