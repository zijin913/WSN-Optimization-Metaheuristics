
function ObjVal = rastrigin(x)
dim=size(x,2);
ObjVal = dim * 10 + sum(((x .* x) - 10 * cos(2 * pi * x))')';
     
return