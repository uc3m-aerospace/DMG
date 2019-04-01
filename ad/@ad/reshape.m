function y = reshape(x,varargin);

% AD implementation of reshape.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

l = length(varargin);
if (l==0)
    error('Reshape requires at least two input arguments');
elseif (l==1),
    v = varargin{1};
    if isa(v,'ad'),
        y.value = reshape(x.value,v.value);
    else
        y.value = reshape(x.value,v);
    end;
    y.derivative = x.derivative;
    y.nderivs = x.nderivs;
else
    xValue = x.value;
    v = [];
    for i=1:l;
        if isa(varargin{i},'ad'),
            v = [v.value varargin{i} ];
        else
            v = [v varargin{i}];
        end;
    end;
    yValue = reshape(xValue,v);
    y.value = yValue;
    y.derivative = x.derivative;
    y.nderivs = x.nderivs;
end;

y = class(y,'ad');

