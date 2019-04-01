function y = asind(x);

% AD implementation of asind.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = asin(x.value)*180/pi;
outerDerivative = (180/pi)./sqrt(1-full(x.value).^2);
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
