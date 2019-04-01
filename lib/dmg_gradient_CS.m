function[G]=gradient_CS(x,mysetup)

%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
G = mysetup.Gzeros;

if strcmp(mysetup.parallel,'yes');
    
        parfor k=1:mysetup.numvars;
            ypert = y + mysetup.deltaxmat(:,k);
            G(:,k) = mysetup.objective(ypert,mysetup);
        end;
        
else
    
        for k=1:mysetup.numvars;
            ypert = y + mysetup.deltaxmat(:,k);
            G(:,k) = mysetup.objective(ypert,mysetup);
        end;
      
end


G = imag(G/mysetup.hpert)*mysetup.invDx;

G = sparse(G);

