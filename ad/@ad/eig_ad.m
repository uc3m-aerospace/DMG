function [V,D] = eig(A)

% AD implementation of the eigen values of a matrix 
      n = size(A.value,1);
      Anderivs    = A.nderivs;
      sizeValue = size(A.value);
      sizeDerivative = [sizeValue Anderivs];
      Aderivative = reshape(full(A.derivative),sizeDerivative);
      
      % Case: A.value is Hermitian
    if ( norm(A.value-A.value',inf)<1e-14 )
        [Vx,Dx] = eig(A.value);
          Ddx = zeros(n,Anderivs);
         for j = 1:Anderivs
             for i = 1:n;
          Ddx(i,j) = Vx(:,i)'*Aderivative(:,:,j)*Vx(:,i);
             end
         end

        if ( nargout==2 ) % eigenvectors are expected
          Vdx = zeros(n,n);
          
          % real and distinct case
          for i=1:n
            r = - ( A.derivative-Ddx(i,i)*eye(n) )*Vx(:,i);
            r = Vx'*r;
            s = zeros(n,1);
            for j=1:i-1
              s(j) = r(j)/(Dx(j,j)-Dx(i,i));
            end
            for j=i+1:n
              s(j) = r(j)/(Dx(j,j)-Dx(i,i));
            end
            Vdx(:,i) = Vx*s;  % will leave out Vx(:,i) component
          end
        end
        
      else % non-Hermitian case
          
        [Vx,Dx] = eig(A.value);
         Ddx = zeros(n,Anderivs);
         for j = 1:Anderivs
             for i = 1:n;
          Ddx(i,j) = Vx(:,i)'*Aderivative(:,:,j)*Vx(:,i);
             end
         end
          %Ddx = reshape (Ddx, n*n, Anderivs);

        if ( nargout==2 ) % eigenvectors are expected
          Vdx = zeros(n,n);
          error ('Non-Hermitian case for eigen vectors not implemented')
        end
         
    end      
      if ( nargout==1 )
        V.value = diag(Dx);
        V.derivative = Ddx;
        V.nderivs  = A.nderivs;
        V = class(V,'ad');
      else
        V.value = Vx;
        V.derivative = Vdx;
        V.nderivs = A.nderivs;
        V = class(V,'ad');
        
        D.value = diag(Dx);
        D.derivative = diag(Ddx);
        D.nderivs = A.nderivs;
        D = class(V,'ad');
      end
      
      
      
      
      
      
      
      
 