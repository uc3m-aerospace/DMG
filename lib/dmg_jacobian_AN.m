function[J]=jacobian_AN(x,mysetup)

%% Unscale the decision variables
y   = x./mysetup.column_scales;
[C,Jac] = mysetup.constraints(y,mysetup);
J   = sparse(Jac*mysetup.invDx);
J(1:mysetup.numnonlin,:) = mysetup.DF*J(1:mysetup.numnonlin,:);
J   = sparse (J+ mysetup.Alinear_augmented);