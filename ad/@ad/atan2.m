function z = atan2(x,y);

% AD implementation of atans.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

ztemp   = atan2(x.value,y.value);
z       = atan(x./y);
z.value = ztemp;
