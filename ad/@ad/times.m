function z=times(x,y);

% AD implementation of times.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(x,'ad'),
    sizeX = size(x.value);
else
    sizeX = size(x);
end;
if isa(y,'ad'),
    sizeY = size(y.value);
else
    sizeY = size(y);
end;
if ~isequal(sizeX,sizeY) && ~isequal(prod(sizeX),1) && ~isequal(prod(sizeY),1),
    error('Dimensions of x and y must be equal'),
end;

if isa(x,'double') && isa(y,'ad')
    z.value = x.*y.value;
    sizeyValue = size(y.value);
    nDerivatives = y.nderivs;
    if prod(sizeX)==1, % x is a scalar
        z.derivative = x*y.derivative;
    elseif prod(sizeY)==1, % y is a scalar
        z.derivative = x(:)*y.derivative;
    else % Both x and y are matrices
        xValue = repmat(x(:),1,nDerivatives);
        z.derivative = xValue.*y.derivative;
    end;
    z.nderivs = nDerivatives;
elseif isa(x,'ad') && isa(y,'double')
    z.value = x.value.*y;
    sizexValue = size(x.value);
    nDerivatives = x.nderivs;
    if prod(sizeX)==1, % x is a scalar
        z.derivative = y(:)*x.derivative;
    elseif prod(sizeY)==1, % y is a scalar
        z.derivative = y*x.derivative;
    else % Both x and y are matrices
        yValue = repmat(y(:),1,nDerivatives);
        z.derivative = x.derivative.*yValue;
    end;
    z.nderivs = nDerivatives;
else
    z.value = x.value.*y.value;
    sizexValue = size(x.value);
    sizeyValue = size(y.value);
    nDerivatives = y.nderivs;
    if prod(sizeX)==1,
        term1 = x.value*y.derivative;
    elseif prod(sizeY)==1,
        term1 = x.value(:)*y.derivative;
    else
        xValue = repmat(x.value(:),1,nDerivatives);
        term1 = xValue.*y.derivative;
    end;
    if prod(sizeX)==1,
        term2 = y.value(:)*x.derivative;        
    elseif prod(sizeY)==1,
        term2 = x.derivative*y.value;
    else
        yValue = repmat(y.value(:),1,nDerivatives);
        term2 = x.derivative.*yValue;
    end;
    z.derivative = term1+term2;
    z.nderivs = nDerivatives;
end;

z = class(z,'ad');
    
