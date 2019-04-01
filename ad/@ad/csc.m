function y = csc(x);

% AD implementation of csc.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = csc(x.value);
outerDerivative = -csc(full(x.value(:))).*cot(full(x.value(:)));
y = compositeDerivative(x,y,outerDerivative)
y = class(y,'ad');
