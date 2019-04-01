function y=log(x);

% AD implementation of log.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = log(x.value);
outerDerivative = 1./(full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
