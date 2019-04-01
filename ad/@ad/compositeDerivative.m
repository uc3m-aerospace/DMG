function y = compositeDerivative(x,y,outerDerivative)

sizeValue = size(x.value);
nDerivatives = x.nderivs;
leadingDim = prod(sizeValue);
[rows,cols,innerDerivative] = find(x.derivative);
derivative = outerDerivative(rows).*innerDerivative;
y.derivative = sparse(rows,cols,derivative,leadingDim,nDerivatives);
y.nderivs = nDerivatives;
