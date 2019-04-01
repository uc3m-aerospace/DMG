function d = diag(x,k);

% AD implementation of diag.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if nargin==1
    k = 0;
end
d.value = diag(x.value,k);
sizeX = size(x.value);
if isequal(sizeX(1),1) || isequal(sizeX(2),1),
    indices = ones(1,prod(sizeX));
    indices = diag(indices,k);
    d.derivative = zeros(prod(size(indices)),x.nderivs);
    d.derivative(~isequal(indices,0),:) = x.derivative;
else
    indices = diag(reshape( 1:prod(sizeX),sizeX(1),sizeX(2)),k);
    d.derivative = x.derivative(indices(:),:);
end
d.nderivs = x.nderivs;
d = class(d,'ad');
  
