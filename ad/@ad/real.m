function z=real(y);
% AD implementation for function REAL.M;

z.value = real(y.value);
z.derivative = y.derivative;
z.nderivs =y.nderivs;
z = class(z,'ad');