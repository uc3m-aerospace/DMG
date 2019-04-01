function[J]=dmg_jacobian_CS(x,mysetup)
%------------------------------------------------------------------%
% Compute jacobian of constraints function for IPOPT when 
% Complex Step Differenciation is used             
%------------------------------------------------------------------%
%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
J = mysetup.Jaczeros;

if strcmp(mysetup.parallel,'yes');
    
        parfor k=1:mysetup.numvars;
            ypert = y + mysetup.deltaxmat(:,k);
            J(:,k) = mysetup.constraints(ypert,mysetup);
        end;
        
else
    
        for k=1:mysetup.numvars;
            ypert = y + mysetup.deltaxmat(:,k);
            J(:,k) = mysetup.constraints(ypert,mysetup);
        end;
            
end
    
J = imag(J/mysetup.hpert)*mysetup.invDx;
%----------------------%
% Unscale the Jacobian %
%----------------------%
J(1:mysetup.numnonlin,:) = mysetup.DF*J(1:mysetup.numnonlin,:);
J = sparse(J + mysetup.Alinear_augmented);