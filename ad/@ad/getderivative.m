function derivative = getderivative(x);

% Returns derivative of input AD object
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(x,'ad'),
    derivative = x.derivative;
end;
