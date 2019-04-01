function events = minfueleventfun(solevents,setup)

xf = solevents.terminal.state;
p= solevents.parameter(1);

tauf = xf(4);


ef = tauf - 3.32 ; 
events=[ ef];


