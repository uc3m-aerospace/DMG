function y = exp(x)

% AD implementation of exp.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = exp(x.value);
outerDerivative = exp(full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');

