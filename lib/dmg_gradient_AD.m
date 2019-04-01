function[G]=dmg_gradient_AD(x,mysetup)
%------------------------------------------------------------------%
% Compute gradient of objective function for IPOPT when 
% automatic differentiation is used             
%------------------------------------------------------------------%
y   = ad(x./mysetup.column_scales);
Obj = mysetup.objective(y,mysetup);
G   = sparse(getderivative(Obj)*mysetup.invDx);


