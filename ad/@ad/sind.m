function y = sind(x);

% AD implementation of sind.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sin(x.value*pi/180);
outerDerivative = pi/180*cos(x.value(:)*pi/180);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
