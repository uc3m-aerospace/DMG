function y = tanh(x);

% AD implementation of tanh.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = tanh(x.value);
outerDerivative = sech(full(x.value)).^2;
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
