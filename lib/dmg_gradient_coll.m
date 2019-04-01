function[G]=gradient_coll(x)
global mysetup
y=ad(x./mysetup.column_scales);
Obj= objective_coll(y);
G= sparse(getderivative(Obj)*mysetup.invDx);


