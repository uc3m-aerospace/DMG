function[C]=dmg_constraints_ipopt(x,mysetup)

%% Unscale the decision variables %

y = x./mysetup.column_scales;

Alinear_augmented = [mysetup.DF*mysetup.sparsity_constant; mysetup.Alinear];
%setup.Alinear_augmented 
C =dmg_constraints(y,mysetup);

%% Scale the constraints
C(1:mysetup.numnonlin) = mysetup.DF*C(1:mysetup.numnonlin);
C = C + Alinear_augmented*y;