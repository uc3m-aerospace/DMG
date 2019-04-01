function z = rdivide(x,y)

% AD implementation of rdivide.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

% z = times(x,y.^(-1));

if 1,
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

if prod(sizeY)==1,
    z = x/y;
    return;
end;

if isa(x,'double') && isa(y,'ad')
    % Derivative is (-x/y^2)*dy
    z.value = x./y.value;
    sizeyValue = size(y.value);
    nDerivatives = y.nderivs;
    if 0,
        xValue = repmat(x(:),1,nDerivatives);
        zValue = repmat(z.value(:),1,nDerivatives);
        yValue = repmat(y.value(:),1,nDerivatives);
        if prod(sizeX)==1,
            z.derivative = -(x./yValue.*yValue).*y.derivative;
        else
  	    z.derivative = -(xValue./(yValue(:).*yValue(:))).*y.derivative;
        end;
    end;
    xoverySquaredAll = -z.value./y.value;
    [rowsy,colsy,innerDerivativey] = find(y.derivative);
    xoverySquared = reshape(xoverySquaredAll(rowsy),size(innerDerivativey));
    z.derivative = sparse(rowsy,colsy,xoverySquared.*innerDerivativey,prod(size(y.value)),nDerivatives);
    z.derivative = reshape(z.derivative,prod(size(y.value)),nDerivatives);
    z.nderivs = nDerivatives;
elseif isa(x,'ad') && isa(y,'double')
    z.value = x.value./y;
    sizexValue = size(x.value);
    nDerivatives = x.nderivs;
    if 0,
        yValue = repmat(y(:),1,nDerivatives);
        z.derivative = x.derivative./yValue;
    end;
    if prod(sizeY)==1,
        oneoveryAll = 1./y(:);
        z.derivative = oneoveryAll*x.derivative;
    else
        oneoveryAll = 1./y(:);
        [rowsx,colsx,innerDerivativex] = find(x.derivative);
        oneovery = reshape(oneoveryAll(rowsx),size(innerDerivativex));
        z.derivative = sparse(rowsx,colsx,oneovery.*innerDerivativex,prod(size(x.value)),nDerivatives);
    end;
    z.derivative = reshape(z.derivative,prod(size(x.value)),nDerivatives);
    z.nderivs = nDerivatives;
else
    z.value = x.value./y.value;
    sizexValue = size(x.value);
    sizeyValue = size(y.value);
    nDerivatives = y.nderivs;
    if 0,
        xValue = repmat(x.value(:),1,nDerivatives);
        yValue = repmat(y.value(:),1,nDerivatives);
        Numerator = yValue.*x.derivative-xValue.*y.derivative;
        Denominator = yValue.^2;
        z.derivative = Numerator./Denominator;
        z.nderivs = nDerivatives;
    end;
    if prod(sizeX)==1,
        prodsizeY = prod(sizeY);
        yValueAll = sparse(y.value(:));
        oneoverySquared = 1./(yValueAll.*yValueAll);
        Numerator = yValueAll*x.derivative-x.value*y.derivative;
        [rowsNum,colsNum,valsNum] = find(Numerator);
        z.derivative = sparse(rowsNum,colsNum,valsNum.*reshape(oneoverySquared,size(valsNum)),prodsizeY,nDerivatives);
    else
        prodsizeY = prod(sizeY);
        xValue = sparse(1:prodsizeY,1:prodsizeY,x.value(:));
        yValue = sparse(1:prodsizeY,1:prodsizeY,y.value(:));
        oneOverYSquared = sparse(1:prodsizeY,1:prodsizeY,1./(y.value(:).*y.value(:)));
        xDerivative = x.derivative;
        yDerivative = y.derivative;
        Numerator = yValue*xDerivative-xValue*yDerivative;
        zDerivative = oneOverYSquared*Numerator;
        z.derivative = reshape(zDerivative,prodsizeY,nDerivatives);
    end;
    z.derivative = reshape(z.derivative,prod(size(x.value)),nDerivatives);
    z.nderivs = nDerivatives;
end;

z = class(z,'ad');

end;
