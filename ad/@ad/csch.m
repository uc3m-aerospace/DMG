function y = csch(x);

% AD implementation of csch.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = csch(x.value);
outerDerivative = -coth(full(x.value)).*csch(full(x.value));
y = compositeDerivative(x,y,outerDerivative)
y = class(y,'ad');
