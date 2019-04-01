function y = secd(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = sec(x.value*pi/180);
outerDerivative = pi/180*sec(full(x.value(:))*pi/180).*tan(full(x.value(:))*pi/180);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
