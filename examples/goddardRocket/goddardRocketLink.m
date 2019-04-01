function link = goddardRocketLink(sol,setup);


xf_left = sol.left.state;
p_left = sol.left.parameter;
left_phase = sol.left.phase;
x0_right = sol.right.state;
p_right = sol.right.parameter;
right_phase = sol.right.phase;

link = x0_right-xf_left;

