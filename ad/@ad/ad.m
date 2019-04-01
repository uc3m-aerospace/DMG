function y = ad(x);

% ----------------------------------------------------%
% Constructor for an Automatic Differentiation Object.
% Code written by Ilyssa Sanders and Anil V. Rao. 
% File Creation Date:  January 2009.
% ----------------------------------------------------%
if isa(x,'ad')
    y = x;
    return
elseif isa(x,'double')
    sx = size(x);
    prodsx = prod(sx);
    sdx = [sx prodsx];
    I = eye(prodsx);
    y.value = full(x);
    y.derivative = sparse(I);
    y.nderivs    = prodsx;
end;

y = class(y,'ad');
