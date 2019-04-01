function y = repmat(x,m,n)

y.value = repmat(x.value,m,n);
y.derivative = repmat(x.derivative,m*n,1);
y.nderivs = x.nderivs;
y = class(y,'ad');
