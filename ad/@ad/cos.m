function y = cos(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = cos(x.value);
outerDerivative = -sin(x.value(:));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
