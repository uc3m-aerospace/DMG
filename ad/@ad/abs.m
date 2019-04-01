function y = abs(x)
% In Package myAD - Automatic Differentiation
% by Martin Fink, June 2006
% martinfink 'at' gmx.at

    temp = sign(x.value);
    y.derivative = temp(:,ones(size(x.derivative,2),1)).*x.derivative;
    y.value = abs(x.values);
    y.nderivs = x.nderivs;
    y = class(y,'ad');