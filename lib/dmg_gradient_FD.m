function[G]=gradient_FD(x,mysetup)

%--------------------------------%
% Unscale the decision variables %
%--------------------------------%
y = x./mysetup.column_scales;
%-----------------------------------------------------------------%
% Compute functions based on unscaled value of decision variables %
%-----------------------------------------------------------------%
G  = mysetup.Gzeros;
G0 = mysetup.objective(y,mysetup); 

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
    
G = (G-G0)/mysetup.hpert*mysetup.invDx;

G = sparse(G);