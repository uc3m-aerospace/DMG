function [dae,Ddae] = chemicalProcessDae(soldae,setup);

t = soldae.time;
s = soldae.state;
u = soldae.control;
p = soldae.parameter;

x = s(:,1);
z = s(:,2);

xdot = x.*z;
zdot = (-z+u).*10;

dae = [xdot zdot];

if nargout == 2
    xdotx = z;
    xdotz = x;
    xdotu =  zeros(size(t));
    xdott =  zeros(size(t));
    zdotx =  zeros(size(t));
    zdotz = -10*ones(size(t));
    zdotu =  10*ones(size(t));
    zdott =  zeros(size(t));
    Ddae = [xdotx xdotz xdotu xdott; zdotx zdotz zdotu zdott];
end
