function y = acot(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = acot(x.value);
outerDerivative = -1./(1+(full(x.value).^2));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
