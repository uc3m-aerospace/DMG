function z = mpower(x,y);

% AD implementation of mpower.m
% Code written by Ilyssa Sanders and Anil V. Rao
% March 2009.

% This function only works for positive scalar integer exponents
% and is NOT configured for general double-precision exponents yet.  
sY = size(y);
if ~isequal(size(y),[1 1]) || ~isequal(round(y),y),
    error('Matrix Power for AD Objects Must Have a Scalar Integer Exponent');
end;
z.value = mpower(x.value,y);
% Check to see if y is even or odd
if y>=0,
    if isequal(y,0),
        z.value = eye(size(x.value));
        z.derivative = zeros(size(x.derivative));
        z.nderivs = x.nderivs;
        z = class(z,'ad');
    elseif y==1,
        z = x;
    elseif y==2,
        z = x*x;
    elseif isequal(y,2*floor(y/2)),
        yhalf = y/2;
        zhalf = x^yhalf;
        z = zhalf*zhalf;
    else 
        y1 = floor(y/2);
        y2 = ceil(y/2);
        z1 = x^y1;
        z2 = x^y2;
        z = z1*z2;
    end;        
else
    error('Matrix Power Only Defined for Positive Integers');
end;
