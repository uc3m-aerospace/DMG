function value = getvalue(x);

% Returns value of input AD object
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(x,'ad'),
  value = x.value;
end;