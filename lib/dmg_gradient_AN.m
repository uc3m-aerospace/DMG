function[G]=gradient_AN(x,mysetup)

y   = x./mysetup.column_scales;
[Obj,G] = mysetup.objective(y,mysetup);
G   = sparse(G*mysetup.invDx);