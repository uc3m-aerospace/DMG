function x = ctranspose(x);

% AD implementation of ctranspose.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if ~isreal(x.value),
    error('Automatic Differentiation is for Real-Valued Functions Only');
end
sizeX = size(x.value);
x.value = x.value';
if ~(prod(sizeX)==1),
    indices = reshape(1:prod(sizeX),sizeX(1),sizeX(2))';
    x.derivative = x.derivative(indices,:);
end

