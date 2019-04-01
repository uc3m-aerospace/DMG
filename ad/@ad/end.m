function e = end(a,i,j)

if j==1,
    e = length(a.value(:));
else
    e = size(a.value,i);
end
