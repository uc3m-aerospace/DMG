function y = cot(x);

% AD implementation of cot.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = cot(x.value);
outerDerivative = -csc(full(x.value(:))).^2;
y = compositeDerivative(x,y,outerDerivative)
y = class(y,'ad');
