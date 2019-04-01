function[J]=jacobian_AD(x,mysetup)

%% Unscale the decision variables
y=ad(x./mysetup.column_scales);
Jac= mysetup.constraints(y,mysetup);
JJ = getderivative(Jac);
J= sparse(getderivative(Jac)*mysetup.invDx);
J(1:mysetup.numnonlin,:) = mysetup.DF*J(1:mysetup.numnonlin,:);
J = sparse (J + mysetup.Alinear_augmented);


