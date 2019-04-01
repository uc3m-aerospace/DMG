function y = sign(x);

% AD implementation of sign.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sign(x.value);
y.derivative = zeros(size(x.derivative));
y.nderivs = x.nderivs;
y = class(y,'ad');
