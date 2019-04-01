function y = sech(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sech(x.value);
outerDerivative = sech(full(x.value(:))).*tanh(full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
