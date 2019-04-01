function z = eq(x,y);

% AD implementation of eq.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if ~isa(x,'ad'),
    z = (x == y.value);
elseif ~isa(y,'ad'),
    z = (x.value == y);
else
    z = (x.value == y.value);
end;
