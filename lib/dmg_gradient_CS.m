function[G]=dmg_gradient_CS(x,mysetup)
%------------------------------------------------------------------%
% Compute gradient of objective function for IPOPT when 
% complex Step differentiation is used             
%------------------------------------------------------------------%
% DMG Copyright (c) David Morante González                         %
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
G = mysetup.Gzeros;

if strcmp(mysetup.parallel,'yes')
    
        parfor k=1:mysetup.numvars
            ypert = y + mysetup.deltaxmat(:,k);
            G(:,k) = mysetup.objective(ypert,mysetup);
        end
        
else
    
        for k=1:mysetup.numvars
            ypert = y + mysetup.deltaxmat(:,k);
            G(:,k) = mysetup.objective(ypert,mysetup);
        end
      
end


G = imag(G/mysetup.hpert)*mysetup.invDx;

G = sparse(G);

