%--------------------------------------
% BEGIN: function brachistochroneCost.m
%--------------------------------------
function [Mayer,Lagrange]=brachistochroneCost(sol,setup);

tf = sol.terminal.time;
t  = sol.time;

Mayer = tf;
Lagrange = zeros(size(t));

