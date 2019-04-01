function[J]=dmg_jacobian_FD(x,mysetup)
%------------------------------------------------------------------%
% Compute jacobian of constraints function for IPOPT when 
% Forward Finite DIfferences is used             
%------------------------------------------------------------------%
%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
J           = mysetup.Jaczeros;
J0          = mysetup.constraints(y,mysetup);
n           = mysetup.numvars;
deltaxmat   = mysetup.deltaxmat;
constraints = mysetup.constraints;

if strcmp(mysetup.parallel,'yes');
    
    parfor k=1:n;
        ypert = y + deltaxmat(:,k);
        J(:,k) = constraints(ypert,mysetup);
    end;
    
else
    
    for k=1:n;
        ypert = y + deltaxmat(:,k);
        J(:,k) = constraints(ypert,mysetup);
    end;
     
end

J = (J-repmat(J0,1,mysetup.numvars))/mysetup.hpert*mysetup.invDx;
%----------------------%
% Unscale the Jacobian %
%----------------------%
J(1:mysetup.numnonlin,:) = mysetup.DF*J(1:mysetup.numnonlin,:);
J = sparse(J + mysetup.Alinear_augmented);












