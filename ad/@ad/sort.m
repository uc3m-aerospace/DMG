function varargout = sort(x)
% In Package AD

[val, idx] = sort(x.value);
x.value = val;
x.derivative = x.derivative(idx,:);
varargout{1} = x;
if (nargout>1)
    varargout{2} = idx;
end
