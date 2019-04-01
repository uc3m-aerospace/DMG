function y = acsch(x);

% Implementation of ACSCH function for AD Objects
% Authors of Code:  Ilyssa Sanders and Anil V. Rao
% Date:  January 2009

y.value = acsch(x.value);
outerDerivative = -1./(full(x.value(:)).*(1+full(x.value).^2));
y = compositeDerivative(x,y,outerDerivative);
y = class(y,'ad');
