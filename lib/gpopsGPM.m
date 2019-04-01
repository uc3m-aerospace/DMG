function Gauss = gpopsGPM(n);
%------------------------------------------------------------------%
% Compute the Gauss points, weights, and differentiation matrix for%
% use in the Gauss pseudospectral method.  This code is divided    %
% into two parts:                                                  %
%   Part 1:  Compute the Legendre-Gauss points and weights         %
%   Part 2:  Compute the differentiation matrix for the Gauss      %
%            pseudospectral method.                                %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%

%------------------------------------------------------------------%
% Part 1:  Get Legendre-Gauss points and Legendre-Gauss weights.   %
% The code below taken and adapted from the Appendix of David      %
% Benson's 2004 MIT Ph.D. Thesis.  For details, see the following  %
% reference:                                                       %    
%    Benson, D. A., A Gauss Pseudospectral Transcription for       %
%    Optimal Control, Ph.D. Thesis, Department of Aeronautics and  %
%    Astronautics, MIT, November 2004.                             %
%------------------------------------------------------------------%
Gauss = gpopsGaussPointsWeights(n);
Gauss_Plus_Init = [-1; Gauss.Points];
n = n+1;

%------------------------------------------------------------------%
% Part 2:  Get the GPM differentiation matrix.  The code below is  %
% taken from the 2004 MIT Ph.D. Thesis of David A. Benson.  See    %
% the following reference:                                         %
%    Benson, D. A., A Gauss Pseudospectral Transcription for       %
%    Optimal Control, Ph.D. Thesis, Department of Aeronautics and  %
%    Astronautics, MIT, November 2004.                             %
%------------------------------------------------------------------%
D = zeros(n,n);
for j = 1:n;
    for i = 1:n;
        prod = 1;
        sum = 0;
        if j == i
            for k = 1:n
                if k~=i
                    sum = sum+1/(Gauss_Plus_Init(i)-Gauss_Plus_Init(k));
                end
            end
            D(i,j) = sum;
        else
            for k = 1:n
                if (k~=i) && (k~=j)
                    prod = prod*(Gauss_Plus_Init(i)-Gauss_Plus_Init(k));
                end
            end
            for k = 1:n
                if k~=j
                    prod = prod/(Gauss_Plus_Init(j)-Gauss_Plus_Init(k));
                end
            end
            D(i,j) = prod;
        end
    end
end
D = D(2:end,:);
Gauss.Differentiation_Matrix = D;
diagD = diag(diag(D,1));
Gauss.Differentiation_Matrix_Diag = zeros(size(D));
Gauss.Differentiation_Matrix_Diag(:,2:end) = diagD;

function Gauss = gpopsGaussPointsWeights(n)
%------------------------------------------------------------------%
% Compute the Legendre-Gauss points and Legendre-Gauss weights for %
% a given value of n.  The code below is adapted from the code     %
% given in the following reference:                                %
% Weideman, J. A. C. and Reddy, S. C., "A MATLAB Differentiation   %
% Matrix Suite," ACM Transactions on Mathematical Software,        %
% Vol. 26, No. 4, December 2000, pp. 465-519.                      %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
Gauss_Points  = gpopsGaussPoints(n);
Leg_Poly_Mat  = legendre(n+1,Gauss_Points);
Leg_Poly      = Leg_Poly_Mat(1,:)';
Leg_Poly_Dot  = -(n+1).*Leg_Poly./(1-Gauss_Points.^2);
Gauss_Weights = (2./(1-Gauss_Points.^2))./Leg_Poly_Dot.^2;
Gauss.Points  = sort(Gauss_Points);
Gauss.Weights = Gauss_Weights;

function Gauss_Points = gpopsGaussPoints(N)
%------------------------------------------------------------------%
% Compute the Legendre-Gauss points for a given value of n.  It is %
% noted that the Legendre-Gauss points are the eigenvalues of the  %
% tridiagonal Jacobi matrix.  The code below is adapted from the   %
% code given in the following reference:                           %
% Weideman, J. A. C. and Reddy, S. C., "A MATLAB Differentiation   %
% Matrix Suite," ACM Transactions on Mathematical Software,        %
% Vol. 26, No. 4, December 2000, pp. 465-519.                      %
%------------------------------------------------------------------%
% GPOPS Copyright (c) Anil V. Rao, Geoffrey T. Huntington, David   %
% Benson, Michael Patterson, Christopher Darby, & Camila Francolin %
%------------------------------------------------------------------%
indices       = 1:(N-1);
subdiagonal   = indices./sqrt(4*indices.*indices-1);
Jacobi_Matrix = diag(subdiagonal,-1)+diag(subdiagonal,1);
Gauss_Points  = sort(eig(sparse(Jacobi_Matrix)));
