function y = sin(x);

% AD implementation of sin.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sin(x.value);
outerDerivative = cos(full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
