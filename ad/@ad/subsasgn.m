function x = subsasgn(x,s,b)

% AD implementation of subsasgn.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if length(s)>1
    error('Multiple indexing for AD object assignment is not permitted');
end

if 0,
if strcmp(s.type,'()'), % assignment x(i) = b
    x.value(s.subs{:}) = b.value;
    sizexValue = size(x.value);
    indices = reshape(1:prod(sizexValue),sizexValue);
    indices = indices(s.subs{:});
    xDerivative = zeros(prod(sizexValue),b.nderivs);
    xDerivative(indices(:),:) = b.derivative;
    x.derivative = xDerivative;
    x.nderivs = b.nderivs;
    if ~isa(x,'ad')
        x = class(x,'ad');
    end;
end;
end;
if 1,
if strcmp(s.type,'()'), % assignment x(i) = b
    checkxEmpty = isempty(x);
    if isempty(b)
        x.value(s.subs{:}) = [];
        values = zeros(size(x.value));
        values(s.subs{:}) = 1;
        x.derivative(values(:)==1,:) = [];  
        return
    end
    if ~isa(b,'ad'), 
        b = adconstant(b);
    end
    if ~checkxEmpty
        increaseSize = false;
        if isequal(length(s.subs),1),
            if ~isequal(s.subs{1},':')
                if size(x.value,1)==1
                    increaseSize=(s.subs{1}>size(x.value,2));
                else
                    increaseSize=(s.subs{1}>prod(size(x.value)));
                end
                if increaseSize,
                    sizexValue = size(x.value);
                    if isequal(length(sizexValue),2),
                        if all(prod(sizexValue)~=sizexValue)
                            error('Cannot resize matrix by the assignment A(I)=B')
                        end
                    else
                        error('Attempting to Increase Size of Array along Ambiguous Dimension')
                    end
                end
            end
        else                            % multiple index
            for i=1:length(s.subs)
                if ~isequal(s.subs{i},':')
                    increaseSize= (increaseSize | ( s.subs{i} > size(x.value,i) ));
                end
            end
        end
        if increaseSize,
            xValue = x.value;
            xDerivative = x.derivative;
            value = ones(size(x.value));
            value(s.subs{:}) = 0;
            x.value = zeros(size(value));
            if issparse(xDerivative)
                x.derivative = sparse([],[],[],prod(size(value)),b.nderivs,0);
            else
                x.derivative = zeros(prod(size(value)),x.nderivs);
            end
            x.value(value==1) = xValue;
            x.derivative(value(:)==1,:) = xDerivative;
        end
    else
        x.value(s.subs{:}) = 0;
        if issparse(b.derivative)
            x.derivative = sparse([],[],[],prod(size(x.value)),b.nderivs,0);
        else
            x.derivative = zeros(prod(size(x.value)),b.nderivs);
        end
    end
    x.value(s.subs{:}) = b.value;
    value = zeros(size(x.value));
    value(s.subs{:}) = 1;
    index = (value(:)==1);
    if (prod(size(b.value))==1 ) & (sum(index)~=1)
        x.derivative(index,:) = ones(sum(index),1)*b.derivative;
    else
        x.derivative(index,:) = b.derivative;
    end
    if checkxEmpty
        x.nderivs = b.nderivs;
        x = class(x,'ad');
    end
else
    error('invalid referencing of indices for AD object');
end
end;
