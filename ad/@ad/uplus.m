function y=uplus(x);

% AD implementation of uplus.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = (x.value);
outerDerivative = ones(size(full(x.value(:))));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
