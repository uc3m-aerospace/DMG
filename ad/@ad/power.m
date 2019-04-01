function z = power(x,y);

% AD implementation of power.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if isa(y,'double')
    % Case where y is NOT an AD object
    if (prod(size(y))==1) && (isequal(y,round(y))),
        % Special algorithm for integer exponents.
        % This algorithm uses recursion by continuing to divide the
        % exponent y until y reaches the value 0, 1, or 2.  It then
        % multiplies together value of 1, x, and x^2 to get the
        % power of the function when y is an integer.
        if y>=0,
            if isequal(y,0),
                z.value = x.value.^0;
                z.derivative = zeros(size(x.derivative));
                z.nderivs = x.nderivs;
                z = class(z,'ad');
            elseif y==1,
                z = x;
            elseif y==2,
                z = x.*x;
            elseif isequal(y,2*floor(y/2)),
                yhalf = y/2;
                zhalf = x.^yhalf;
                z = zhalf.*zhalf;
            else 
                y1 = floor(y/2);
                y2 = ceil(y/2);
                z1 = x.^y1;
                z2 = x.^y2;
                z = z1.*z2;
            end;        
        else
            if isequal(y,0),
                z.value = x.value.^0;
                z.derivative = zeros(size(x.derivative));
                z.nderivs = x.nderivs;
                z = class(z,'ad');
            elseif y==-1,
                z = 1./x;
            elseif y==-2,
                z = 1./(x.*x);
            elseif isequal(y,2*floor(y/2)),
                yhalf = y/2;
                zhalf = x.^yhalf;
                z = zhalf.*zhalf;
            else 
                y1 = floor(y/2);
                y2 = ceil(y/2);
                z1 = x.^y1;
                z2 = x.^y2;
                z = z1.*z2;
            end;        
        end;
    else
        % When y is not an integer, the best way to compute the
        % power is using exp(y*log(x)).  
        z = exp(y.*log(x));
    end;
else
    % When y is an AD object the best way to compute the power is
    % using exp(y*log(x)).
    z = exp(y.*log(x));
end;
