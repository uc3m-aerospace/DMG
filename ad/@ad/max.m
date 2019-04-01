
function z = max(varargin)
% In Package myAD - Automatic Differentiation
% by Martin Fink, June 2006
% martinfink 'at' gmx.at

    if nargout > 1
        error('Only one output for max implemented');
    end

    if nargin > 1
        x = max(varargin{1}.value, varargin{2}.value);
        idx = (x == varargin{2}.value) + 1;
        z = varargin{idx};
    else
        [x, idx] = max(varargin{1}.value);
        z = varargin{1};
        z.value = z.value(idx);
        z.derivative = z.derivative(idx,:);
    end
