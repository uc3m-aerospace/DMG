function c = cross(a,b,dim)

% AD implementation of cross.m
% Code written by Ilyssa Sanders and Anil V. Rao
% January 2009

if nargin<2,
    error('Need at least two arguments for cross product');
elseif nargin>=2,
    if isa(a,'ad'),
        sizeA = size(a.value);
    else
        sizeA = size(a);
    end;
    if isa(b,'ad'),
        sizeB = size(b.value);
    else
        sizeB = size(b);
    end;
    if ~isequal(sizeA,sizeB),
        error('Matrices must the same size');
    end;
    if ~((isequal(sizeA(1),3) && isequal(sizeB(1),3)) || (isequal(sizeA(2),3) && isequal(sizeB(2),3))),
        error('At least one of the dimensions must be equal to 3');
    end;
    if nargin==3,
        if ~isequal(prod(size(dim)),1),
            error('Third input argument must be a scalar');
        end;
        if (isequal(dim,1) && ~isequal(sizeA(1),3)) || (isequal(dim,2) && ~isequal(sizeA(2),3)),
            error('A and B must be of length 3 in the dimension in which the cross product is taken.');
        end;
    end;
end;

if isequal(sizeA(1),3), % Orientation of data is along rows
    if ~isa(a,'ad'),
        aValue = a;
        bValue = b.value;
        aValueRep = repmat(aValue,1,b.nderivs);
        aValueRep = reshape(aValueRep,sizeA(1),sizeA(2),b.nderivs);
        bValueRep = repmat(bValue,1,b.nderivs);
        bValueRep = reshape(bValueRep,sizeB(1),sizeB(2),b.nderivs);
        c.value = zeros(sizeA(1),sizeA(2));
        c.value(1,:) = aValue(2,:).*bValue(3,:)-aValue(3,:).*bValue(2,:);
        c.value(2,:) = aValue(3,:).*bValue(1,:)-aValue(1,:).*bValue(3,:);
        c.value(3,:) = aValue(1,:).*bValue(2,:)-aValue(2,:).*bValue(1,:);
        bDerivative = reshape(full(b.derivative),sizeB(1),sizeB(2),b.nderivs);
        term1 = aValueRep(2,:,:).*bDerivative(3,:,:)-aValueRep(3,:,:).*bDerivative(2,:,:);
        term2 = aValueRep(3,:,:).*bDerivative(1,:,:)-aValueRep(1,:,:).*bDerivative(3,:,:);
        term3 = aValueRep(1,:,:).*bDerivative(2,:,:)-aValueRep(2,:,:).*bDerivative(1,:,:);
        cDerivative(1,:,:) = term1;
        cDerivative(2,:,:) = term2;
        cDerivative(3,:,:) = term3;
        c.derivative = sparse(reshape(cDerivative,prod(sizeA),b.nderivs));
        c.nderivs = b.nderivs;
    elseif ~isa(b,'ad')
        aValue = a.value;
        bValue = b;
        aValueRep = repmat(aValue,1,a.nderivs);
        aValueRep = reshape(aValueRep,sizeA(1),sizeA(2),a.nderivs);
        bValueRep = repmat(bValue,1,a.nderivs);
        bValueRep = reshape(bValueRep,sizeB(1),sizeB(2),a.nderivs);
        c.value = zeros(sizeA(1),sizeA(2));
        c.value(1,:) = aValue(2,:).*bValue(3,:)-aValue(3,:).*bValue(2,:);
        c.value(2,:) = aValue(3,:).*bValue(1,:)-aValue(1,:).*bValue(3,:);
        c.value(3,:) = aValue(1,:).*bValue(2,:)-aValue(2,:).*bValue(1,:);
        aDerivative = reshape(full(a.derivative),sizeA(1),sizeA(2),a.nderivs);
        term1 = aDerivative(2,:,:).*bValueRep(3,:,:)-aDerivative(3,:,:).*bValueRep(2,:,:);
        term2 = aDerivative(3,:,:).*bValueRep(1,:,:)-aDerivative(1,:,:).*bValueRep(3,:,:);
        term3 = aDerivative(1,:,:).*bValueRep(2,:,:)-aDerivative(2,:,:).*bValueRep(1,:,:);
        cDerivative(1,:,:) = term1;
        cDerivative(2,:,:) = term2;
        cDerivative(3,:,:) = term3;
        c.derivative = sparse(reshape(cDerivative,prod(sizeA),a.nderivs));
        c.nderivs = a.nderivs;
    else
        aValue = a.value;
        bValue = b.value;
        aValueRep = repmat(aValue,1,a.nderivs);
        aValueRep = reshape(aValueRep,sizeA(1),sizeA(2),a.nderivs);
        bValueRep = repmat(bValue,1,a.nderivs);
        bValueRep = reshape(bValueRep,sizeB(1),sizeB(2),a.nderivs);
        c.value = zeros(sizeA(1),sizeA(2));
        c.value(1,:) = aValue(2,:).*bValue(3,:)-aValue(3,:).*bValue(2,:);
        c.value(2,:) = aValue(3,:).*bValue(1,:)-aValue(1,:).*bValue(3,:);
        c.value(3,:) = aValue(1,:).*bValue(2,:)-aValue(2,:).*bValue(1,:);
        aDerivative = reshape(full(a.derivative),sizeA(1),sizeA(2),a.nderivs);
        bDerivative = reshape(full(b.derivative),sizeB(1),sizeB(2),b.nderivs);
        term11 = aValueRep(2,:,:).*bDerivative(3,:,:)+aDerivative(2,:,:).*bValueRep(3,:,:);
        term12 = aValueRep(3,:,:).*bDerivative(2,:,:)+aDerivative(3,:,:).*bValueRep(2,:,:);
        term21 = aValueRep(3,:,:).*bDerivative(1,:,:)+aDerivative(3,:,:).*bValueRep(1,:,:);
        term22 = aValueRep(1,:,:).*bDerivative(3,:,:)+aDerivative(1,:,:).*bValueRep(3,:,:);
        term31 = aValueRep(1,:,:).*bDerivative(2,:,:)+aDerivative(1,:,:).*bValueRep(2,:,:);
        term32 = aValueRep(2,:,:).*bDerivative(1,:,:)+aDerivative(2,:,:).*bValueRep(1,:,:);
        cDerivative(1,:,:) = term11-term12;
        cDerivative(2,:,:) = term21-term22;
        cDerivative(3,:,:) = term31-term32;
        c.derivative = sparse(reshape(cDerivative,prod(sizeA),a.nderivs));
        c.nderivs = a.nderivs;
    end;
else, % Orientation of data is along rows
    if ~isa(a,'ad'),
        aValue = a;
        bValue = b.value;
        aValueRep = repmat(aValue,1,b.nderivs);
        aValueRep = reshape(aValueRep,sizeA(1),sizeA(2),b.nderivs);
        bValueRep = repmat(bValue,1,b.nderivs);
        bValueRep = reshape(bValueRep,sizeB(1),sizeB(2),b.nderivs);
        c.value = zeros(sizeA(1),sizeA(2));
        c.value(:,1) = aValue(:,2).*bValue(:,3)-aValue(:,3).*bValue(:,2);
        c.value(:,2) = aValue(:,3).*bValue(:,1)-aValue(:,1).*bValue(:,3);
        c.value(:,3) = aValue(:,1).*bValue(:,2)-aValue(:,2).*bValue(:,1);
        bDerivative = reshape(full(b.derivative),sizeB(1),sizeB(2),b.nderivs);
        term11 = aValueRep(:,2,:).*bDerivative(:,3,:);
        term12 = aValueRep(:,3,:).*bDerivative(:,2,:);
        term21 = aValueRep(:,3,:).*bDerivative(:,1,:);
        term22 = aValueRep(:,1,:).*bDerivative(:,3,:);
        term31 = aValueRep(:,1,:).*bDerivative(:,2,:);
        term32 = aValueRep(:,2,:).*bDerivative(:,1,:);
        cDerivative(:,1,:) = term11-term12;
        cDerivative(:,2,:) = term21-term22;
        cDerivative(:,3,:) = term31-term32;
        c.derivative = sparse(reshape(cDerivative,prod(sizeA),b.nderivs));
        c.nderivs = b.nderivs;
    elseif ~isa(b,'ad')
        aValue = a.value;
        bValue = b;
        aValueRep = repmat(aValue,1,a.nderivs);
        aValueRep = reshape(aValueRep,sizeA(1),sizeA(2),a.nderivs);
        bValueRep = repmat(bValue,1,a.nderivs);
        bValueRep = reshape(bValueRep,sizeB(1),sizeB(2),a.nderivs);
        c.value = zeros(sizeA(1),sizeA(2));
        c.value(:,1) = aValue(:,2).*bValue(:,3)-aValue(:,3).*bValue(:,2);
        c.value(:,2) = aValue(:,3).*bValue(:,1)-aValue(:,1).*bValue(:,3);
        c.value(:,3) = aValue(:,1).*bValue(:,2)-aValue(:,2).*bValue(:,1);
        aDerivative = reshape(full(a.derivative),sizeA(1),sizeA(2),a.nderivs);
        term11 = -aDerivative(:,2,:).*bValueRep(:,3,:);
        term12 = -aDerivative(:,3,:).*bValueRep(:,2,:);
        term21 = -aDerivative(:,3,:).*bValueRep(:,1,:);
        term22 = -aDerivative(:,1,:).*bValueRep(:,3,:);
        term31 = -aDerivative(:,1,:).*bValueRep(:,2,:);
        term32 = -aDerivative(:,2,:).*bValueRep(:,1,:);
        cDerivative(:,1,:) = term11-term12;
        cDerivative(:,2,:) = term21-term22;
        cDerivative(:,3,:) = term31-term32;
        c.derivative = sparse(reshape(cDerivative,prod(sizeA),a.nderivs));
        c.nderivs = a.nderivs;
    else
        aValue = a.value;
        bValue = b.value;
        aValueRep = repmat(aValue,1,a.nderivs);
        aValueRep = reshape(aValueRep,sizeA(1),sizeA(2),a.nderivs);
        bValueRep = repmat(bValue,1,b.nderivs);
        bValueRep = reshape(bValueRep,sizeB(1),sizeB(2),a.nderivs);
        c.value = zeros(sizeA(1),sizeA(2));
        c.value(:,1) = aValue(:,2).*bValue(:,3)-aValue(:,3).*bValue(:,2);
        c.value(:,2) = aValue(:,3).*bValue(:,1)-aValue(:,1).*bValue(:,3);
        c.value(:,3) = aValue(:,1).*bValue(:,2)-aValue(:,2).*bValue(:,1);
        aDerivative = reshape(full(a.derivative),sizeA(1),sizeA(2),a.nderivs);
        bDerivative = reshape(full(b.derivative),sizeB(1),sizeB(2),b.nderivs);
        term11 = aValueRep(:,2,:).*bDerivative(:,3,:)-aDerivative(:,2,:).*bValueRep(:,3,:);
        term12 = aValueRep(:,3,:).*bDerivative(:,2,:)-aDerivative(:,3,:).*bValueRep(:,2,:);
        term21 = aValueRep(:,3,:).*bDerivative(:,1,:)-aDerivative(:,3,:).*bValueRep(:,1,:);
        term22 = aValueRep(:,1,:).*bDerivative(:,3,:)-aDerivative(:,1,:).*bValueRep(:,3,:);
        term31 = aValueRep(:,1,:).*bDerivative(:,2,:)-aDerivative(:,1,:).*bValueRep(:,2,:);
        term32 = aValueRep(:,2,:).*bDerivative(:,1,:)-aDerivative(:,2,:).*bValueRep(:,1,:);
        cDerivative(1,:,:) = term11-term12;
        cDerivative(2,:,:) = term21-term22;
        cDerivative(3,:,:) = term31-term32;
        c.derivative = sparse(reshape(cDerivative,prod(sizeA),a.nderivs));
        c.nderivs = a.nderivs;
    end;
end;

c = class(c,'ad');
