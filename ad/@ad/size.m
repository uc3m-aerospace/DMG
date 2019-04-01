function varargout = size(varargin);

% AD implementation of size.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if nargin==1,
    x = varargin{1};
    nDerivatives = x.nderivs;
    xValue = size(x.value);
    Derivative = sparse(zeros(prod(xValue),nDerivatives));
elseif nargin==2,
    x = varargin{1};
    dim = varargin{2};
    nDerivatives = x.nderivs;
    xValue = size(x.value,dim);
    xDerivative = sparse(zeros(xValue,nDerivatives));
end;

if nargout == 0,
    varargout{1} = xValue;
elseif nargout==1,
    xout.value = xValue;
    xout.derivative = sparse(prod(size(xValue)),nDerivatives);
    xout.nderivs = nDerivatives;
    xout = class(xout,'ad');
    varargout{1} = xout;
else
    for i=1:nargout;
        xout.value = xValue(i);
        xout.derivative = sparse(1,nDerivatives);
        xout.nderivs = nDerivatives;
        xout = class(xout,'ad');
        varargout{i} = xout;
        clear xout;
    end;
end;
