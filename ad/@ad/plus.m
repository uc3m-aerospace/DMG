function z=plus(x,y);

% AD implementation of plus.m
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
    z.value = x+y.value;
    sizeyValue = size(y.value);
    nDerivatives = y.nderivs;
    leadingDim = prod(sizeyValue);
    [rows,cols,yDerivative] = find(y.derivative);
    derivative = yDerivative(:);
    z.derivative = sparse(rows,cols,derivative,leadingDim,nDerivatives);
    z.nderivs = nDerivatives;
elseif isa(x,'ad') && isa(y,'double')
    z.value = x.value+y;
    sizexValue = size(x.value);
    nDerivatives = x.nderivs;
    leadingDim = prod(sizexValue);
    [rows,cols,xDerivative] = find(x.derivative);
    derivative = xDerivative(:);
    z.derivative = sparse(rows,cols,derivative,leadingDim,nDerivatives);
    z.nderivs = nDerivatives;
else
    z.value = x.value+y.value;
    sizexValue = size(x.value);
    sizeyValue = size(y.value);
    nDerivatives = y.nderivs;
    leadingDim = prod(sizexValue);
    if ~isequal(size(x.derivative),size(y.derivative)),
        [rowsx,cols,xDerivative] = find(x.derivative);
        [rowsy,cols,yDerivative] = find(y.derivative);
        if isequal(size(xDerivative,2),size(yDerivative,2)),
            if size(xDerivative,1)==1,
                xDerivative = repmat(xDerivative,size(yDerivative,1),1);
                rows = rowsy;
            elseif size(yDerivative,1)==1,
                yDerivative = repmat(yDerivative,size(xDerivative,1),1);
                rows = rowsx;
            end;
        else
            error('Cannot add incompatible ad objects');
        end;
        derivative = yDerivative(:)+xDerivative(:);
        z.derivative = sparse(rows,cols,derivative,leadingDim,nDerivatives);
        z.nderivs = nDerivatives;
    else
        tderivative = x.derivative+y.derivative;
        [rows,cols,derivative] = find(tderivative);
        z.derivative = sparse(rows,cols,derivative,leadingDim,nDerivatives);
        z.nderivs = nDerivatives;
    end;        
end;

z = class(z,'ad');
    
