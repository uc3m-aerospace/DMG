function y = acscd(x);

% AD implementation of cos.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = acsc(x.value)*180/pi;
outerDerivative = -(180/pi)./(abs(full(x.value(:))).*sqrt(full(x.value).^2-1));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
