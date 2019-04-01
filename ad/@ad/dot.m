function c = dot(a,b,dim)

% AD implementation of dot.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if nargin<2,
    error('Need at least two arguments for dot product');
elseif nargin>=2,
    if isa(a,'ad'),
        sizeA = size(a.value);
    else
        sizeA = size(a);
    end;
    if isa(b,'ad'),
        sizeB = size(b.value);
    else
        sizeB = size(b);
    end;
    if ~isequal(sizeA,sizeB),
        error('Matrices must the same size');
    end;
    if ~isequal(sizeA,sizeB),
        error('Matrices must be the same size');
    end;
    if nargin==3,
        if ~isequal(prod(size(dim)),1),
            error('Third input argument must be a scalar');
        end;
    else
        % If the dimension along which the dot product is not
        % given, the default to taking the dot product
        % column-wise.
        dim = 1;
    end;
end;
% Now take the dot product
if isequal(dim,1),
    c = sum(a.*b,1);
else
    c = sum(a.*b,2);
end;
