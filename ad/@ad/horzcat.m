function y = horzcat(varargin)

% AD implementation of horzcat.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

% Find the first AD object in varargin
ii = cellfun('isclass',varargin,'ad');
ii = find(ii);
if isempty(ii),
    error('At least one input argument must be an AD object');
else
    ii = ii(1);
    nDerivatives = varargin{ii}.nderivs;
end;
currarg = varargin{1};
if ~isempty(currarg),
    if ~(isa(currarg,'ad') || isa(currarg,'double')),
        error('Input must either be an AD object or a double');
    end;
    if isa(currarg,'double'),
        currarg = adconstant_(currarg,nDerivatives);
    end;
%    nDerivatives = currarg.nderivs;
    y.value = currarg.value;
    yDerivative = currarg.derivative;
    sizeCurrArg = size(currarg.value);
else
    y.value = [];
    yDerivative = [];
%    nDerivatives = varargin{2}.nderivs;
end;
for i=2:length(varargin);
    currarg = varargin{i};
    if ~isempty(currarg),
        if isa(currarg,'ad')
            y.value = [y.value, currarg.value];
            sizeCurrArg = size(currarg.value);
            currDerivative = getderivative(currarg);
            yDerivative = [yDerivative; currDerivative];
        elseif isa(currarg,'double');
            currarg = adconstant_(currarg,nDerivatives);
            y.value = [y.value, currarg.value];
            sizeCurrArg = size(currarg.value);
            currDerivative = sparse(prod(sizeCurrArg),nDerivatives);
            yDerivative = [yDerivative; currDerivative];
        end;
    end;
end;
sizeY = size(y.value);
y.derivative = reshape(yDerivative,prod(sizeY),nDerivatives);
y.nderivs = nDerivatives;
y = class(y,'ad');

function y = adconstant_(x,nderivs);

% Initialization of a Constant as an AD Object
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(x,'ad'),
    y = x;
    return;
else
    y.value = x;
    leadingDim = prod(size(x));
    y.derivative = sparse([],[],[],leadingDim,nderivs);
end;
y.nderivs = nderivs;

y = class(y,'ad');

