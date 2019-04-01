function y = regroup(x,m)
%  REGROUP:  regroups a matrix of size (p*m)-by-n (where each m-by-n 
%  matrix is stored one atop the other) into a matrix of size
%  m-by-(n*p), where each m-by-n matrix is stored side-by-side.
%
%  USAGE:
%    y = regroup(x,m)
%
%    x - m-by-n matrices stacked p high, thus p*m rows
%    m - must provide desired m, used to compute p which is rows/m
%    y - m-by-n matrices stacked p left to right, thus n*p columns
%
%   NOTE: There's no error checking, so be sure to provide an m
%   that divides the number of rows into an integer, p.
% 
%   Example: 
%       m=4; n=5; p=7;             % 7, 4x5 stacked atop each other
%       x=reshape(1:m*n*p,n,m*p)'  % 28 by 5
%       y=regroup(x,m)             % 4 by 35
%
%  Written by: Maj Tim Jorris, USAF Test Pilot School, Jan 2009
%  This file is part of the AD code and is used by permission of
%  Major Tim Jorris
%  
%  See also RESHAPE

[mp,n]=size(x); p=mp/m;
ywrong=reshape(x,m,n*p); % m by (p*n), but in the wrong order
id=reshape(1:n*p,p,n)';  % provides correct counting with for loop
id=reshape(id,1,n*p);    % places in index shape
y=ywrong(:,id);          % correct size, and correct order
    
