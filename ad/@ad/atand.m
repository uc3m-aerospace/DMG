function y = atand(x);

% AD implementation of atand.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = atan(x.value)*180/pi;
outerDerivative = (180/pi)./(1+full(x.value).^2);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
