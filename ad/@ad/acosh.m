function y = acosh(x);

% AD implementation of acosh.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = acosh(x.value);
outerDerivative = 1./sqrt(full(x.value).^2-1);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
