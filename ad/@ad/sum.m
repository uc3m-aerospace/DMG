function y = sum(x,dim);

% AD implementation of sum.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

sizeX = size(x.value);
if nargin==1,
    if sizeX(1)==1,
        dim = 2;
    else
        dim = 1;
    end;
end;

nDerivatives = x.nderivs;
if any(sizeX==1),    
    if ~(((sizeX(1)==1) & (dim==1)) | ((sizeX(2)==1) & (dim==2))),
        y.value = sum(x.value);
        yDerivative = reshape(x.derivative,sizeX(1),sizeX(2)*nDerivatives);
        yDerivative = sum(yDerivative);
    else
        y.value = x.value;
        yDerivative = x.derivative;
    end;
else
    y.value = sum(x.value,dim);
    if dim==1,
        yDerivative = sparse(repmat(1:sizeX(2),sizeX(1),1),1:prod(sizeX),1)*x.derivative;
    else
        yDerivative = repmat(speye(sizeX(1)),1,sizeX(2))*x.derivative;
    end;
end;
y.derivative = yDerivative;
y.nderivs = nDerivatives;

y = class(y,'ad');
