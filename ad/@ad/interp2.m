function zi = interp2(x,y,z,xi,yi,varargin)
 
% AD implementation of interp2.m
% Code written by David Benson
% Aug 2011
%
% Note:  This function currently only allows 'linear' interpolation,
%        The other methods will produce an error
%       (see documentation for MATLAB function INTERP2)
% Inputs and Outputs
%      x: known vector of x's for interpolation
%      y: known vector of y's for interpolation
%      z: known 2D matrix of z's for interpolation
%      xi: object of AD class
%      yi: object of AD class
%      method: 'linear'
%      zi: object of AD class
 
error(nargchk(5,7,nargin,'struct')); % allowing for an ExtrapVal
 
if nargin > 5 && ~strcmpi(varargin{1},'linear')
  error('GPOPS:interp2:method','only ''linear'' interpolation supported for automatic derivative.');
end
 
% get ad value
if isa(xi,'ad')
    xiCol = xi.value;
else
    xiCol = xi;
end
if isa(yi,'ad')
    yiCol = yi.value;
else
    yiCol = yi;
end
 
if ~all(size(xiCol) == size(yiCol)) && ...
    ~isscalar(xiCol) && ~isscalar(yiCol)
  error('GPOPS:interp2:method','vectors xi and yi must be the same size for automatic derivative.');
end
 
% get interp2 value
if nargin < 7
  zi.value = interp2(x,y,z,xiCol,yiCol);
else
  zi.value = interp2(x,y,z,xiCol,yiCol,'linear',varargin{2});
end
 
% reset to columns
xiCol = xiCol(:);
yiCol = yiCol(:);
 
% compute x direction linear derivative
if isa(xi,'ad')
  if ~isvector(x)
    % assumes x formed using meshgrid
    xCol = x(1,:).';
  else
    xCol = x(:);
  end
  n = length(xCol);
  [~,kx] = histc(xiCol,xCol);
  kx(xiCol<xCol(1) | ~isfinite(xiCol)) = 1;
  kx(xiCol>=xCol(n)) = n-1;
 
  if nargin < 7
    zXlow = interp2(x,y,z,xCol(kx),yiCol);
    zXhigh = interp2(x,y,z,xCol(kx+1),yiCol);
  else
    zXlow = interp2(x,y,z,xCol(kx),yiCol,'linear',varargin{2});
    zXhigh = interp2(x,y,z,xCol(kx+1),yiCol,'linear',varargin{2});
  end
  outerDerivX = (zXhigh(:) - zXlow(:))./(xCol(kx+1)-xCol(kx));
 
  nDerivatives = xi.nderivs;
  term1 = xi.derivative.*repmat(outerDerivX(:),1,nDerivatives);
   
end
 
% compute y direction linear derivative
if isa(yi,'ad')
  if ~isvector(y)
    % assumes y formed using meshgrid
    yCol = y(:,1);
  else
    yCol = y(:);
  end
  n = length(yCol);
  [~,ky] = histc(yiCol,yCol);
  ky(yiCol<yCol(1) | ~isfinite(yiCol)) = 1;
  ky(yiCol>=yCol(n)) = n-1;
 
  if nargin < 7
    zYlow = interp2(x,y,z,xiCol,yCol(ky));
    zYhigh = interp2(x,y,z,xiCol,yCol(ky+1));
  else
    zYlow = interp2(x,y,z,xiCol,yCol(ky),'linear',varargin{2});
    zYhigh = interp2(x,y,z,xiCol,yCol(ky+1),'linear',varargin{2});
  end
 
  outerDerivY = (zYhigh(:) - zYlow(:))./(yCol(ky+1)-yCol(ky));
 
  nDerivatives = yi.nderivs;
  term2 = yi.derivative.*repmat(outerDerivY(:),1,nDerivatives);
   
end
 
% combine derivatives
if ~isa(xi,'ad')
  zi.derivative = term2;
elseif ~isa(yi,'ad')
  zi.derivative = term1;
else
  zi.derivative = term1 + term2;
end
zi.nderivs = nDerivatives;
   
zi = class(zi,'ad');