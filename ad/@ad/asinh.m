function y = asinh(x);

% AD implementation of asinh.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = asinh(x.value);
outerDerivative = 1./sqrt(1+full(x.value).^2);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
