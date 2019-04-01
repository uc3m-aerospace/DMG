function daeout = leeBioreactorDae(soldae,setup);

t = soldae.time;;
s = soldae.state;
u = soldae.control;
p = soldae.parameter;

x1 = s(:,1);
x2 = s(:,2);
x3 = s(:,3);
x4 = s(:,4);
x5 = s(:,5);
x6 = s(:,6);
x7 = s(:,7);

u1 = s(:,8);
u2 = s(:,9);

c1 = 100; 
c2 = 0.51; 
c3 = 4.0;

t1 = 14.35+x3+((x3).^2/111.5);
t2 = 0.22+x5;
t3 = x6+0.22./t2.*x7;

g1 = x3./t1.*(x6+x7.*(0.22./t2));
g2 = 0.233*x3./t1.*((0.0005+x5)./(0.022+x5));
g3 = 0.09*x5./(0.034+x5);
 
x1dot = u1+u2;
x2dot = g1.*x2-(u1+u2).*x2./x1;
x3dot = u1./x1.*c1-(u1+u2).*x3./x1-g1.*x2/c2;
x4dot = g2.*x2-(u1+u2).*x4./x1;
x5dot = u2*c3./x1-(u1+u2).*x5./x1;
x6dot = -g3.*x6;
x7dot = g3.*(1-x7);
x8dot = u(:,1);
x9dot = u(:,2);

daeout = [x1dot x2dot x3dot x4dot x5dot x6dot x7dot x8dot x9dot];

