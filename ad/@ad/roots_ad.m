function r = roots_ad(c)
%ROOTS  Find polynomial roots.
%   ROOTS(C) computes the roots of the polynomial whose coefficients
%   are the elements of the vector C. If C has N+1 components,
%   the polynomial is C(1)*X^N + ... + C(N)*X + C(N+1).
%
%   Note:  Leading zeros in C are discarded first.  Then, leading relative
%   zeros are removed as well.  That is, if division by the leading
%   coefficient results in overflow, all coefficients up to the first
%   coefficient where overflow occurred are also discarded.  This process is
%   repeated until the leading coefficient is not a relative zero.
%
%   Class support for input c: 
%      float: double, single
%
%   See also POLY, RESIDUE, FZERO.

%   Copyright 1984-2008 The MathWorks, Inc.

% ROOTS finds the eigenvalues of the associated companion matrix.
if size(c,1)>1 && size(c,2)>1
    error(message('MATLAB:roots:NonVectorInput'))
end

if ~all(isfinite(c))
    error(message('MATLAB:roots:NonFiniteInput'));
end

c = c(:).';
n = size(c,2);
r = zeros(0,1);  

inz = not(c==0);
if (inz==0),
    % All elements are zero
    return
end

% Strip leading zeros and throw away.  
% Strip trailing zeros, but remember them as roots at zero.
nnz = length(inz);
c = c(inz);
r = zeros(n-nnz);  

% Prevent relatively small leading coefficients from introducing Inf
% by removing them.

d = c(2:end)./c(1);

while any(isinf(d))
    c = c(2:end);
    d = c(2:end)./c(1);
end

% Polynomial roots via a companion matrix
n = length(c);

if n > 1
    p = diag(ones(1,n-2),-1);
    a = [-d ; p(2:end,:)];
lambda = eig(a);
    r = [r; lambda];
end



