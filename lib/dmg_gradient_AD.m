function[G]=gradient_AD(x,mysetup)

y   = ad(x./mysetup.column_scales);
Obj = mysetup.objective(y,mysetup);
G   = sparse(getderivative(Obj)*mysetup.invDx);


