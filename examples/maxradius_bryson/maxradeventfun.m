function events = maxradeventfun(solevents,setup)

iphase = solevents.phase;
t0 = solevents.initial.time;
x0 = solevents.initial.state;
tf = solevents.terminal.time;
xf = solevents.terminal.state;

ef = xf(3) - sqrt(1/xf(1));
events=ef;
