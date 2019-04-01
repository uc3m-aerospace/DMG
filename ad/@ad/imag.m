function z=imag(y);
% AD implementation for function imag;

z.value = imag(y.value);
z.derivative = y.derivative;
z.nderivs =y.nderivs;
z = class(z,'ad');