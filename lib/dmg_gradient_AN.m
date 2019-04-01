function[G]=dmg_gradient_AN(x,mysetup)
%------------------------------------------------------------------%
% Compute gradient of objective function for IPOPT when Analytic
% derivatives are provided is used             
%------------------------------------------------------------------%
y   = x./mysetup.column_scales;
[Obj,G] = mysetup.objective(y,mysetup);
G   = sparse(G*mysetup.invDx);