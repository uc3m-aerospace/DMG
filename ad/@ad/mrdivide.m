function z = mrdivide(x,y)

% AD implementation of mrdivide.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(y,'ad'),
    if prod(size(y.value))~=1
        error('Denominator must be a scalar');
    end;
else
    if prod(size(y))~=1
        error('Denominator must be a scalar');
    end;
end;
    

if ~isa(x,'ad'),
    z.value = x/y.value;
    nDerivatives = y.nderivs;
    n = prod(size(x));
    if n==1,
        OneOverDenominator = -x/(y.value.*y.value);
        z.derivative = y.derivative*OneOverDenominator;
    else
      OneOverDenominator = 1/(y.value.*y.value);
      if issparse(y.derivative)
          xValue = sparse(-x(:));
      else
          xValue = -x(:);
      end
      z.derivative = xValue*(y.derivative*OneOverDenominator);
    end
elseif ~isa(y,'ad')
    z.value = x.value/y;
    nDerivatives = x.nderivs;
    z.derivative = x.derivative/y;
    z.nderivs = x.nderivs;
else
    z.value=x.value/y.value;
    nDerivatives = x.nderivs;
    OneOverDenominator = 1/(y.value.*y.value);
    n = prod(size(x.value));
    if n==1
        Numerator = x.derivative*y.value-x.value*y.derivative;
        z.derivative=Numerator*OneOverDenominator;
    else
        if issparse(x.derivative)
            xValue=sparse(x.value(:));
        else
            xValue = x.value(:);
        end
        Numerator = x.derivative*y.value-xValue*y.derivative;
        z.derivative = Numerator*OneOverDenominator;
    end
end
z.nderivs = nDerivatives;
z = class(z,'ad');

