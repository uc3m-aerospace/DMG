function yi = interp4(pp,xi,varargin)

% AD implementation of interp1.m
% Code written by David Benson
% April 2009
%
% Note:  This function currently only allows 'spline' interpolation.  The
%        other methods will produce an error
%       (see documentation for MATLAB function INTERP1)
% Inputs and Outputs
%      x: known vector of x's for interpolation
%      y: known vector of y's for interpolation
%      xi: object of AD class 
%      method: 'spline'
%      yi: object of AD class 

%% Process METHOD in
% YI = INTERP1(X,Y,XI,METHOD,...)
% including explicit specification of the default by an empty input.
if nargin >= 3 && ~isempty(varargin{1})
    method = varargin{1};
else
    method = 'linear';
end

switch method(1)
   case 's'  % 'spline'       

       yi = ppval(pp,xi);

    otherwise % 'nearest', 'linear', 'v5cubic'
       error('GPOPS:interp1:InvalidMethod','Invalid method. Only ''spline'' interpolation supported for automatic derivatives') 
end



