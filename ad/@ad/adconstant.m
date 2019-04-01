function y = adconstant(x);

% Initialization of a Constant as an AD Object
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(x,'ad'),
    y = x;
else
    y.value = x;
    leadingDim = prod(size(x));
    y.derivative = sparse([],[],[],leadingDim,y.nderivs);
end;

y = class(y,'ad');
