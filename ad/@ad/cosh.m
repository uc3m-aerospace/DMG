function y = cosh(x);

% AD implementation of cosh.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = cosh(x.value);
outerDerivative = sinh(full(x.value));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
