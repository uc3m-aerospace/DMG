function [F,G] = snwrapper(x,userfun,nF,varargin)
%function [F,G] = snwrapper(x,userfun);
%   Wrapper to allow variable length arguments in
%   user-supplied functions.
 %% HE CAMBIADO COSAS => ( nargin = 3,4,5 (funciona en 2014b) ; por nargin = 2,3,4 (funciona en 2012) )
 global mysetup
 
if ( nargin == 3 ),
  try
    [F,G] = feval(userfun,x,mysetup);
  catch
    F     = feval(userfun,x,mysetup);
    G     = [];
  end

else
  if ( nargin == 4 ),
    myobj = varargin{1};

    try
      [obj,grad] = feval(myobj,x);

      F          = [ obj; zeros(nF-1,1) ];
      G          = [ grad; zeros(nF-1,n) ];
    catch
      [obj] = feval(myobj,x);

      F     = [ obj; zeros(nF-1,1) ];
      G     = [];
    end

  elseif ( nargin == 5 ),
    myobj   = varargin{1};
    nonlcon = varargin{2};

    try
      [obj,grad]      = feval(myobj,x);
      [c,ceq,dc,dceq] = feval(nonlcon,x);

      z = nF - 1 - size(c,1) - size(ceq,1);
      F = [ obj; c; ceq; zeros(z,1) ];
      G = [ grad; dc; ceq; zeros(z,n) ];

    catch
      [obj]   = feval(myobj,x);
      [c,ceq] = feval(nonlcon,x);

      z = nF - 1 - size(c,1) - size(ceq,1);
      F = [ obj; c; ceq; zeros(z,1) ];
      G = [];
    end
  end
end
