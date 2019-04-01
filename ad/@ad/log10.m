function y = log10(x)

% AD implementation of log.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = log10(x.value);
outerDerivative = 1./(log(10).*full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');

