function y = acotd(x);

% AD implementation of acotd.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = acot(x.value)*180/pi;
outerDerivative = -(180/pi)./(1+(full(x.value).^2));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
