function y = atan(x);

% AD implementation of atan.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = atan(x.value);
outerDerivative = 1./(1+full(x.value).^2);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
