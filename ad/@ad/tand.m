function y = tand(x);

% AD implementation of tand.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = tan(x.value*pi/180);
outerDerivative = pi/180*sec(full(x.value(:))*pi/180).^2;
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
