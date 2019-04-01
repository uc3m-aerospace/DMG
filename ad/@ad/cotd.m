function y = cotd(x);

% AD implementation of cotd.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

y.value = cot(x.value*pi/180);
outerDerivative = -pi/180*csc(full(x.value(:))*pi/180).^2;
y = compositeDerivative(x,y,outerDerivative)
y = class(y,'ad');
