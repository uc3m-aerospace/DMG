function y = sqrt(x);

% AD implementation of sqrt.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sqrt(x.value);
outerDerivative = 1./(2*sqrt(full(x.value(:))));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');

