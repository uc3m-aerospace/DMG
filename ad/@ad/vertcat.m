function y = vertcat(varargin)

% AD implementation of vertcat.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

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
    sizeCurrArg = size(currarg.value);
    yDerivative = currarg.derivative;
    yDerivative = reshape(yDerivative,sizeCurrArg(1),sizeCurrArg(2)*nDerivatives);
    sizeCurrArg = size(currarg.value);
else
    y.value = [];
    yDerivative = [];
%    nDerivatives = varargin{2}.nderivs;
    sizeCurrArg = size([]);
end;
for i=2:length(varargin);
    currarg = varargin{i};
    if ~isempty(currarg),
        if isa(currarg,'ad')
            y.value = [y.value; currarg.value];
            sizeCurrArg = size(currarg.value);
            currDerivative = getderivative(currarg);
            currDerivative = reshape(currDerivative,sizeCurrArg(1),sizeCurrArg(2)*nDerivatives);
            yDerivative = [yDerivative; currDerivative];
        elseif isa(currarg,'double');
            currarg = adconstant_(currarg,nDerivatives);
            y.value = [y.value; currarg.value];
            sizeCurrArg = size(currarg.value);
            currDerivative = sparse(sizeCurrArg(1),sizeCurrArg(2)*nDerivatives);
            yDerivative = [yDerivative; currDerivative];
        end;
    end;
end;
sizeY = size(y.value);
y.derivative = reshape(yDerivative,prod(sizeY),nDerivatives);
y.nderivs = nDerivatives;
y = class(y,'ad');

function y = adconstant_(x,nderivs);

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
