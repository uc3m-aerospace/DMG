function y = cscd(x);

% AD implementation of csc.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = csc(x.value*pi/180);
outerDerivative = -pi/180*csc(full(x.value(:))*pi/180).*cot(full(x.value(:))*pi/180);
y = compositeDerivative(x,y,outerDerivative)
y = class(y,'ad');
