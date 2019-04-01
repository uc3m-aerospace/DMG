function [C] = gpopsObjandCons_coll(x)

global mysetup

C = constraints_coll(x,mysetup);
Cost= objective_coll(x,mysetup);

C=[Cost;C];


