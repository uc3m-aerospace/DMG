function y = sinh(x);

% AD implementation of sinh.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sinh(x.value);
outerDerivative = cosh(full(x.value));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
