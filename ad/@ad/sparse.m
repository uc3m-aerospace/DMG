function x = sparse(x);

% AD implementation of sparse.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if ~(prod(size(x.value))==1),
    a.value = sparse(a.value);
end
if ~prod(size(a.derivative))==1),
    a.derivative = sparse(a.derivative);
end
