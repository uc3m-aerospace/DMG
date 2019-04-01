function y = subsref(x,s)

% AD implementation of subsref.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if ~isa(x,'ad'),
    y = subsref(x,s(1));
elseif strcmp(s(1).type,'()'),
    y.value = x.value(s(1).subs{:});
    index = reshape(1:prod(size(x.value)),size(x.value));
    index = index(s(1).subs{:});
    xDerivative = x.derivative;
    y.derivative = xDerivative(index(:),1:x.nderivs);
    y.nderivs = x.nderivs;
    y = class(y,'ad');
elseif strcmp(s(1).type,'.'),
    if strcmp(s(1).subs,'value')
        y = x.value;
    elseif strcmp(s(1).subs,'derivative')
        sizeValue = prod(size(x.value));
        nderivatives = x.nderivs;
        y = reshape(x.derivative,[sizeValue nderivatives]);
    elseif strcmp(s(1).subs,'mid')
        y = mid(x);
    else
        error('invalid subscript reference for ad')
    end
else
    error('invalid index reference for ad')
end
s = s(2:end);
