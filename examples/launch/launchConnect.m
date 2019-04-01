function [connect Dconnect] = launchConnect(sol,setup);

xf_left = sol.left.state;
p_left = sol.left.parameter;
left_phase = sol.left.phase;
x0_right = sol.right.state;
p_right = sol.right.parameter;
right_phase = sol.right.phase;

connect = x0_right-xf_left;


% avoid calc of derivs in not necessary
if nargout == 2
    Dconnect = [-eye(length(xf_left)), eye(length(x0_right))];
end