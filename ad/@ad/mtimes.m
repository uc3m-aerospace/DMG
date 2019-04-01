function z=mtimes(x,y);

% Implementation of Matrix Multiply for AD Objects
% Authors of Code:  Ilyssa Sanders and Anil V. Rao
% Date:  January 2009

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
if ~isequal(sizeX(2),sizeY(1)) && ~isequal(prod(sizeX),1) && ~isequal(prod(sizeY),1),
    error('Inner matrix dimensions must agree');
end;
if (length(sizeX)>2) || (length(sizeY)>2),
    error('mtimes only works for dimension two or lower');
end;

if prod(sizeX)==1,
    z = x.*y;
    return;
end;
if prod(sizeY)==1,
    z = y.*x;
    return;
end;

if isa(x,'double') && isa(y,'ad')
    % COMPUTE X*YDERIVATIVE where X is assumed to be CONSTANT
    z.value = x*y.value;
    nDerivatives = y.nderivs;
    yDerivative = reshape(y.derivative,sizeY(1),sizeY(2)*nDerivatives);
    zDerivative = sparse(x)*yDerivative;
    sizeZ = [sizeX(1) sizeY(2)];
    zDerivative = reshape(zDerivative,prod(sizeZ),nDerivatives);
    z.derivative = zDerivative;
    z.nderivs = nDerivatives;
elseif isa(x,'ad') && isa(y,'double')
    % COMPUTE XDERIVATIVE*Y where Y is assumed to be CONSTANT
    z.value = x.value*y;
    nDerivatives = x.nderivs;
    xDerivative = reshape(x.derivative,sizeX(1),sizeX(2)*nDerivatives);
    xDerivative = regroup(xDerivative.',sizeX(2)).';
    sizeZ = [sizeX(1) sizeY(2)];
    zDerivative = xDerivative*sparse(y);
    zDerivative = regroup(zDerivative,sizeX(1));
    z.derivative = reshape(zDerivative,prod(sizeZ),nDerivatives);
    z.nderivs = nDerivatives;
else
    % COMPUTE X*YDERIVATIVE+XDERIVATIVE*Y
    z.value = x.value*y.value;
    nDerivatives = x.nderivs;
    yDerivative = reshape(y.derivative,sizeY(1),sizeY(2)*nDerivatives);
    term1 = sparse(x.value)*yDerivative;
    sizeZ = [sizeX(1) sizeY(2)];
    xDerivative = reshape(x.derivative,sizeX(1),sizeX(2)*nDerivatives);
    xDerivative = regroup(xDerivative.',sizeX(2)).';
    sizeZ = [sizeX(1) sizeY(2)];
    zDerivative = xDerivative*sparse(y.value);
    term2 = regroup(zDerivative,sizeX(1));
    zDerivative = term1 + term2;
    z.derivative = reshape(zDerivative,prod(sizeZ),nDerivatives);
    z.nderivs = nDerivatives;
end;

z = class(z,'ad');

