%------------------------------------
% BEGIN: function brysonDenhamDae.m
%------------------------------------
function [dae,Ddae] = brysonDenhamDae(sol,setup);
%
t = sol.time;
x = sol.state;
u = sol.control;
x1dot = x(:,2);
x2dot = u;
x3dot = u.^2/2;
path = x(:,1);
dae = [x1dot x2dot x3dot path];
%

if nargout == 2
    x1dotx1 = zeros(size(t));
    x1dotx2 = ones(size(t));
    x1dotx3 = zeros(size(t));
    x1dotu  = zeros(size(t)); 
    x1dott  = zeros(size(t));
    x2dotx1 = zeros(size(t));
    x2dotx2 = zeros(size(t));
    x2dotx3 = zeros(size(t));
    x2dotu  = ones(size(t));
    x2dott  = zeros(size(t));
    x3dotx1 = zeros(size(t));
    x3dotx2 = zeros(size(t));
    x3dotx3 = zeros(size(t));
    x3dotu  = u;
    x3dott  = zeros(size(t));
    pathx1 = ones(size(t));
    pathx2 = zeros(size(t));
    pathx3 = zeros(size(t));
    pathu  = zeros(size(t));
    patht  = zeros(size(t));
    Ddae = [x1dotx1 x1dotx2 x1dotx3 x1dotu x1dott;
            x2dotx1 x2dotx2 x2dotx3 x2dotu x2dott;
	        x3dotx1 x3dotx2 x3dotx3 x3dotu x3dott;
	        pathx1  pathx2  pathx3  pathu  patht];
end

%------------------------------------
% END: function brysonDenhamDae.m
%------------------------------------
