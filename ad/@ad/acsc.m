function y = acsc(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = acsc(x.value);
outerDerivative = -1./(abs(full(x.value(:))).*sqrt(full(x.value).^2-1));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
