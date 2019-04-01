function z = zeros(varargin);

% AD implementation of zeros.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if nargin==1,
    xarg = varargin;
    x = varargin{1};
    nDerivatives = x.nderivs;
    z.value = zeros(x.value);
    z.derivative = sparse(prod(x.value),nDerivatives);
    z.nderivs = x.nderivs;
    z = class(z,'ad');
else
    for i=1:nargin;
        x = varargin{i};
        value(i) = x.value;
    end;
    z.value = zeros(value);
    nDerivatives = x.nderivs;
    z.derivative = zeros(prod(value),nDerivatives);
    z.nderivs = nDerivatives;
    z = class(z,'ad');
end;

    
