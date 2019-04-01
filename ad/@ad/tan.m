function y = tan(x);

% AD implementation of tan.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = tan(x.value);
outerDerivative = sec(full(x.value(:))).^2;
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
