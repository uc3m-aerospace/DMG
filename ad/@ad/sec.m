function y = sec(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sec(x.value);
outerDerivative = sec(full(x.value(:))).*tan(full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
