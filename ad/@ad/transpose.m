function x = transpose(x);

% AD implementation of transpose.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

sizeX = size(x.value);
x.value = x.value.';
if ~(prod(sizeX)==1),
    indices = reshape(1:prod(sizeX),sizeX(1),sizeX(2))';
    x.derivative = x.derivative(indices,:);
  end
