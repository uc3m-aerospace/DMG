function y = coth(x);

% AD implementation of coth.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = coth(x.value);
outerDerivative = -csch(full(x.value)).^2;
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
